#!/usr/bin/env python3
"""lcd_node.py — I2C 16x2 LCD에 순찰 상태를 표시한다.

[로봇에서 실행]  (LCD는 물리적으로 로봇에만 있으므로 여기서만 돈다)
  python3 ~/launch/lcd_node.py

[배선 — 2026-09-02 실측]
  SDA = GPIO2 (물리 3번 핀), SCL = GPIO3 (물리 5번 핀), VCC = 5V, GND = GND
  I2C 주소 0x27 (PCF8574 백팩). i2cdetect -y 1 로 확인 가능.
  ⚠️ "GPIO 2번"은 물리적 2번 핀이 아니라 3번 핀이다 — 이걸 헷갈려서 처음에
     i2cdetect 에 아무것도 안 잡혔다(물리 2번은 5V 전원 자리).

[입력]  /fire/status         (String)  "FIRE" 가 들어있으면 화재 화면
        /restricted/status   (String)  "ALERT" 가 들어있으면 침입 경고 화면
        /helmet/status       (String)  "hold" 가 들어있으면 안전모 경고 화면
        (아무것도 없으면 평상시 화면)

[표시 문구 — 사용자 사양서 그대로]
  평상시     PATROL ACTIVE   / AREA IS SECURE
  사람 감지  PERSON DETECTED / WARNING: ALERT
  화재       !! FIRE ALARM !!/ EVACUATE NOW
  안전모     NO HELMET!      / WEAR HELMET NOW

[우선순위]
화재 > 안전모 > 침입 > 평상시. 화재는 사람이 죽을 수 있는 상황이라 다른 무엇도
덮어쓰지 못하게 최상위로 둔다.

[한글을 안 쓰는 이유]
이 LCD(HD44780 호환)는 한글 폰트가 없어 한글을 넣으면 깨진 기호가 나온다.
음성 안내는 한국어로 하고(스피커), 화면은 로마자로 적는다 — helmet_node 의
annotate() 가 cv2 폰트 때문에 로마자를 쓰는 것과 같은 이유다.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from RPLCD.i2c import CharLCD
except ImportError:
    CharLCD = None

SCREENS = {
    "fire":      ("!! FIRE ALARM !!", "EVACUATE NOW"),
    "helmet":    ("NO HELMET!", "WEAR HELMET NOW"),
    "intrusion": ("PERSON DETECTED", "WARNING: ALERT"),
    "idle":      ("PATROL ACTIVE", "AREA IS SECURE"),
}

# 소화기 점검 결과는 내용이 매번 달라서 SCREENS 에 못 넣는다 — 받은 정보로
# 두 줄을 만들어 번갈아 보여준다(16x2 에 제조년월·교체년월·책임자가 다 안 들어간다).
INSPECT_HOLD_SEC = 12.0     # 점검 화면을 이만큼 유지한다
INSPECT_PAGE_SEC = 3.0      # 이 간격으로 다음 페이지로 넘긴다

# 상태가 이 시간(초) 동안 갱신되지 않으면 그 경고는 끝난 것으로 본다.
# 각 노드가 상태를 주기적으로 내지 않고 "바뀔 때만" 내는 경우가 있어서,
# 화면이 옛 경고에 영원히 붙어있지 않게 하는 안전장치다.
STALE_SEC = 12.0


class LcdNode(Node):
    def __init__(self):
        super().__init__("lcd_node")

        self.declare_parameter("address", 0x27)
        self.declare_parameter("port", 1)
        self.declare_parameter("cols", 16)
        self.declare_parameter("rows", 2)
        self.declare_parameter("refresh_sec", 0.5)

        if CharLCD is None:
            self.get_logger().error(
                "RPLCD 가 없다 — pip3 install RPLCD 후 로봇에서 실행할 것")
            raise SystemExit(1)

        self.lcd = CharLCD(
            i2c_expander="PCF8574",
            address=int(self.get_parameter("address").value),
            port=int(self.get_parameter("port").value),
            cols=int(self.get_parameter("cols").value),
            rows=int(self.get_parameter("rows").value),
            dotsize=8,
        )
        self.lcd.clear()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/fire/status", self.on_fire, qos)
        self.create_subscription(String, "/restricted/status", self.on_restricted, qos)
        self.create_subscription(String, "/helmet/status", self.on_helmet, qos)
        self.create_subscription(String, "/extinguisher/inspect_status",
                                 self.on_inspect, qos)

        self.fire_at = 0.0
        self.intrusion_at = 0.0
        self.helmet_at = 0.0
        self.inspect_at = 0.0
        self.inspect_pages = []     # 점검 결과를 보여줄 (1줄, 2줄) 목록
        self.shown = None

        self.create_timer(float(self.get_parameter("refresh_sec").value), self.refresh)
        self.get_logger().info(
            f"lcd_node 시작 — I2C 0x{int(self.get_parameter('address').value):02x}")

    # ---------------- 상태 수신 ----------------
    def on_fire(self, msg: String):
        if "FIRE" in msg.data.upper():
            self.fire_at = time.time()
        else:
            self.fire_at = 0.0      # clear/해제 메시지

    def on_restricted(self, msg: String):
        if "ALERT" in msg.data.upper():
            self.intrusion_at = time.time()
        else:
            self.intrusion_at = 0.0

    def on_helmet(self, msg: String):
        # helmet_node 는 "hold (...)" / "released" 를 낸다
        if msg.data.startswith("hold"):
            self.helmet_at = time.time()
        else:
            self.helmet_at = 0.0

    def on_inspect(self, msg: String):
        """소화기 점검 결과를 받아 LCD 페이지로 만든다.

        extinguisher_inspect_node 가 내는 형식:
          "점검 <이름> verdict=.. qr=.. mfg=.. exp=.. mgr=.. days_left=.."
        점검이 아닌 상태 문구("대기 — ...")는 무시한다.
        """
        text = msg.data
        if not text.startswith("점검 "):
            return
        f = {}
        for tok in text.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                f[k] = v

        def ascii_only(s, limit=16):
            # HD44780 LCD 에 한글 폰트가 없다 — 로마자·숫자만 남긴다.
            return s.encode("ascii", "ignore").decode()[:limit]

        verdict = f.get("verdict", "?")
        # 판정만 로마자로 바꿔 보여준다(한글은 LCD 에서 깨진다)
        vmap = {"정상": "OK", "이상": "ABNORMAL", "판정불가": "CANNOT JUDGE",
                "부재": "MISSING"}
        pages = [
            ("EXTINGUISHER", f"GAUGE: {vmap.get(verdict, '?')}"),
            ("MFG / EXP", ascii_only(f"{f.get('mfg','?')} {f.get('exp','?')}")),
        ]
        days = f.get("days_left")
        if days and days != "?":
            pages.append(("REPLACE IN", f"{days} DAYS"))
        mgr = ascii_only(f.get("mgr", ""))
        # 책임자 이름이 한글이면 로마자로 남는 게 없다 — 그럴 때는 이 페이지를 뺀다
        # (빈 줄만 나오면 LCD 가 고장난 것처럼 보인다). 웹에는 한글로 제대로 나온다.
        pages.append(("MANAGER", mgr if mgr.strip() else "SEE WEB"))
        self.inspect_pages = pages
        self.inspect_at = time.time()

    # ---------------- 화면 갱신 ----------------
    def pick_screen(self):
        now = time.time()

        def fresh(ts):
            return ts > 0.0 and (now - ts) < STALE_SEC

        if fresh(self.fire_at):
            return "fire"
        if fresh(self.helmet_at):
            return "helmet"
        if fresh(self.intrusion_at):
            return "intrusion"
        # 점검 결과는 경고보다 낮은 우선순위 — 화재 중에 소화기 정보를 띄우면 안 된다.
        if self.inspect_pages and (now - self.inspect_at) < INSPECT_HOLD_SEC:
            return "inspect"
        return "idle"

    def refresh(self):
        key = self.pick_screen()

        if key == "inspect":
            # 여러 페이지를 번갈아 보여준다(16x2 에 다 안 들어간다).
            elapsed = time.time() - self.inspect_at
            idx = int(elapsed // INSPECT_PAGE_SEC) % len(self.inspect_pages)
            line1, line2 = self.inspect_pages[idx]
            tag = f"inspect:{idx}"
            if tag == self.shown:
                return
            self.lcd.clear()
            self.lcd.write_string(line1[:16])
            self.lcd.crlf()
            self.lcd.write_string(line2[:16])
            self.shown = tag
            self.get_logger().info(f"LCD -> [{tag}] {line1} / {line2}")
            return

        if key == self.shown:
            return                  # 같은 화면이면 다시 쓰지 않는다(깜빡임 방지)
        line1, line2 = SCREENS[key]
        self.lcd.clear()
        self.lcd.write_string(line1[:16])
        self.lcd.crlf()
        self.lcd.write_string(line2[:16])
        self.shown = key
        self.get_logger().info(f"LCD -> [{key}] {line1} / {line2}")

    def destroy_node(self):
        try:
            self.lcd.clear()
            self.lcd.write_string("PATROL OFFLINE")
        except Exception:                                       # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LcdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
