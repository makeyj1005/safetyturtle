#!/usr/bin/env python3
"""gpio_io_node.py — 로봇(Pi4) GPIO에 직결된 센서·액추에이터를 ROS2 토픽으로 잇는다.

[로봇에서 실행]  (GPIO는 물리적으로 로봇에만 있으므로 여기서만 돈다)
  python3 ~/launch/gpio_io_node.py

[핀 배치 — 2026-09-02, 사용자 제공]
  GPIO23  불꽃 감지 센서 (디지털 입력)
  GPIO24  능동부저 (디지털 출력)
  GPIO25  LED (디지털 출력)
  GPIO27  인체감지센서 PIR (디지털 입력)
  (LCD는 I2C, GPIO2/3 — 이 노드가 아니라 lcd_node.py 에서 따로 다룸)

[입력]  /buzzer/set  (Bool)  True/False — 능동부저 켜고 끄기
        /led/set     (Bool)  True/False — LED 켜고 끄기
[출력]  /flame/detected  (Bool)  불꽃 감지 여부 (상태 바뀔 때만 발행)
        /pir/detected    (Bool)  인체감지 여부 (상태 바뀔 때만 발행)

[센서 극성]
불꽃센서·PIR 모듈은 제품마다 감지 시 HIGH 인지 LOW 인지 다르다. 기본은 "HIGH=감지"로
두고, 실측해서 반대면 --ros-args -p flame_active_high:=false 처럼 뒤집을 것.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

PIN_FLAME = 23
PIN_BUZZER = 24
PIN_LED = 25
PIN_PIR = 27


class GpioIoNode(Node):
    def __init__(self):
        super().__init__("gpio_io_node")

        # 2026-09-02 실측(감도 조절 후 라이터로 확인): 평상시 LOW, 불꽃 감지시 HIGH.
        # ⚠️ 감도 조절 나사를 돌려야 반응한다 — 조절 전에는 계속 LOW로 고정돼 있어서
        #    "배선이 잘못됐나" 오진하기 쉽다. 불을 켠 상태로 나사를 돌려서 맞출 것.
        self.declare_parameter("flame_active_high", True)
        self.declare_parameter("pir_active_high", True)
        self.declare_parameter("poll_hz", 10.0)
        # 화재는 오탐 대가가 크니(불필요한 대피 소동) 순간 튐을 걸러낸다 — 이만큼
        # 연속으로 같은 값이 나와야 실제로 바뀐 것으로 본다. poll_hz=10 이면 0.3초.
        self.declare_parameter("flame_debounce_n", 3)

        if GPIO is None:
            self.get_logger().error(
                "RPi.GPIO 가 없다 — 로봇(Pi)에서 실행하는 게 맞는지 확인할 것"
            )
            raise SystemExit(1)

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_FLAME, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PIN_PIR, GPIO.IN)
        GPIO.setup(PIN_BUZZER, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_LED, GPIO.OUT, initial=GPIO.LOW)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_flame = self.create_publisher(Bool, "/flame/detected", qos)
        self.pub_pir = self.create_publisher(Bool, "/pir/detected", qos)
        self.create_subscription(Bool, "/buzzer/set", self.on_buzzer, qos)
        self.create_subscription(Bool, "/led/set", self.on_led, qos)

        self.last_flame = None
        self.last_pir = None
        self.flame_candidate = None
        self.flame_streak = 0

        hz = float(self.get_parameter("poll_hz").value)
        self.create_timer(1.0 / hz, self.poll)

        self.get_logger().info(
            f"gpio_io_node 시작 — 불꽃=GPIO{PIN_FLAME}, PIR=GPIO{PIN_PIR}, "
            f"부저=GPIO{PIN_BUZZER}, LED=GPIO{PIN_LED}"
        )

    def poll(self):
        flame_high = bool(self.get_parameter("flame_active_high").value)
        pir_high = bool(self.get_parameter("pir_active_high").value)

        flame_raw = GPIO.input(PIN_FLAME) == GPIO.HIGH
        flame = flame_raw if flame_high else not flame_raw
        need_n = int(self.get_parameter("flame_debounce_n").value)
        if flame == self.flame_candidate:
            self.flame_streak += 1
        else:
            self.flame_candidate = flame
            self.flame_streak = 1
        if self.flame_streak >= need_n and flame != self.last_flame:
            self.last_flame = flame
            m = Bool()
            m.data = flame
            self.pub_flame.publish(m)
            self.get_logger().warn(f"불꽃 감지 상태 변경: {flame}")

        pir_raw = GPIO.input(PIN_PIR) == GPIO.HIGH
        pir = pir_raw if pir_high else not pir_raw
        if pir != self.last_pir:
            self.last_pir = pir
            m = Bool()
            m.data = pir
            self.pub_pir.publish(m)

    def on_buzzer(self, msg: Bool):
        GPIO.output(PIN_BUZZER, GPIO.HIGH if msg.data else GPIO.LOW)

    def on_led(self, msg: Bool):
        GPIO.output(PIN_LED, GPIO.HIGH if msg.data else GPIO.LOW)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = GpioIoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # try_shutdown: SIGTERM 으로 죽일 때 이미 shutdown 이 불린 경우가 있어
        # rclpy.shutdown() 을 그냥 부르면 RCLError 를 던진다(다른 노드들과 같은 처리).
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
