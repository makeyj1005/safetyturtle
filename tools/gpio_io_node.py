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

        # 2026-09-02 실측: 평상시(불 없음) HIGH, 감지시 LOW인 모듈이었다 — 기본을 false로.
        self.declare_parameter("flame_active_high", False)
        self.declare_parameter("pir_active_high", True)
        self.declare_parameter("poll_hz", 10.0)

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
        if flame != self.last_flame:
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
