"""Non-blocking ROS 2 servo release controller."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ServoState(str, Enum):
    IDLE = "IDLE"
    PREPARING = "PREPARING"
    READY = "READY"
    RELEASING = "RELEASING"
    RETURNING = "RETURNING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    PWM_STOPPED = "PWM_STOPPED"


@dataclass(frozen=True)
class ServoConfig:
    gpio_pin: int = 18
    safe_angle_deg: float = 0.0
    release_angle_deg: float = 90.0
    min_angle_deg: float = 0.0
    max_angle_deg: float = 180.0
    min_pulse_us: int = 1000
    max_pulse_us: int = 2000
    prepare_duration_sec: float = 0.5
    release_duration_sec: float = 0.8
    return_duration_sec: float = 0.5
    return_after_release: bool = True
    disable_pwm_after_action: bool = True
    allow_repeat: bool = False
    dry_run: bool = True


def validate_config(config: ServoConfig) -> list[str]:
    errors = []
    if config.gpio_pin < 0:
        errors.append("gpio_pin must be non-negative")
    if not (math.isfinite(config.min_angle_deg) and math.isfinite(config.max_angle_deg)):
        errors.append("angle limits must be finite")
    elif config.min_angle_deg >= config.max_angle_deg:
        errors.append("min_angle_deg must be less than max_angle_deg")
    for name, angle in (("safe_angle_deg", config.safe_angle_deg),
                        ("release_angle_deg", config.release_angle_deg)):
        if not math.isfinite(angle) or not config.min_angle_deg <= angle <= config.max_angle_deg:
            errors.append(f"{name} must be within the configured angle range")
    if config.min_pulse_us >= config.max_pulse_us or config.min_pulse_us <= 0:
        errors.append("min_pulse_us must be positive and less than max_pulse_us")
    if config.max_pulse_us <= 0:
        errors.append("max_pulse_us must be positive")
    for name, duration in (("prepare_duration_sec", config.prepare_duration_sec),
                           ("release_duration_sec", config.release_duration_sec),
                           ("return_duration_sec", config.return_duration_sec)):
        if not math.isfinite(duration) or duration < 0:
            errors.append(f"{name} must be non-negative")
    return errors


def angle_to_pulsewidth(angle_deg: float, config: ServoConfig) -> int:
    """Map an angle to pulse width, rejecting angles outside configured limits."""
    if not config.min_angle_deg <= angle_deg <= config.max_angle_deg:
        raise ValueError("angle is outside configured limits")
    fraction = (angle_deg - config.min_angle_deg) / (config.max_angle_deg - config.min_angle_deg)
    return round(config.min_pulse_us + fraction * (config.max_pulse_us - config.min_pulse_us))


class ServoDriver:
    """Small hardware boundary, allowing the state machine to run without GPIO."""

    def __init__(self, pin: int, dry_run: bool, logger: Optional[Callable[[str], None]] = None,
                 pigpio_module=None):
        self.pin = pin
        self.dry_run = dry_run
        self._logger = logger or (lambda message: None)
        self._pi = None
        if not dry_run:
            if pigpio_module is None:
                import pigpio as pigpio_module
            self._pi = pigpio_module.pi()
            if self._pi is None or not self._pi.connected:
                raise RuntimeError("pigpiod connection failed; start pigpiod first")

    def set_pulsewidth(self, pulsewidth: int) -> None:
        if self.dry_run:
            self._logger(f"DRY-RUN servo GPIO{self.pin} pulse={pulsewidth}us")
            return
        result = self._pi.set_servo_pulsewidth(self.pin, pulsewidth)
        if result != 0:
            raise RuntimeError(f"pigpio set_servo_pulsewidth returned {result}")

    def stop(self) -> None:
        if self.dry_run or self._pi is None:
            return
        try:
            self._pi.set_servo_pulsewidth(self.pin, 0)
        finally:
            self._pi.stop()
            self._pi = None


class ServoActionMachine:
    """Timer-driven action logic independent of ROS and real GPIO."""

    def __init__(self, config: ServoConfig, driver: ServoDriver,
                 state_callback: Callable[[str], None], result_callback: Callable[[str], None]):
        errors = validate_config(config)
        if errors:
            raise ValueError("; ".join(errors))
        self.config = config
        self.driver = driver
        self.state_callback = state_callback
        self.result_callback = result_callback
        self.state = ServoState.IDLE
        self.completed = False
        self.phase = None
        self.deadline = 0.0
        self._emit_state(ServoState.IDLE)

    def _emit_state(self, state: ServoState | str) -> None:
        self.state = ServoState(state) if state in ServoState.__members__ else state
        self.state_callback(str(self.state.value if isinstance(self.state, ServoState) else self.state))

    def _move(self, angle: float) -> None:
        self.driver.set_pulsewidth(angle_to_pulsewidth(angle, self.config))

    def command(self, command: str, now: float) -> None:
        command = command.strip().lower()
        if command not in {"prepare", "throw", "reset", "stop"}:
            self.result_callback(f"REJECTED:{command or '<empty>'}:unknown_command")
            return
        if command == "stop":
            try:
                self.driver.stop()
                self.phase = None
                self._emit_state(ServoState.PWM_STOPPED)
                self.result_callback("SUCCESS:stop")
            except Exception as exc:
                self.fail(str(exc))
            return
        if self.phase is not None:
            self._emit_state("BUSY")
            self.result_callback(f"REJECTED:{command}:busy")
            return
        if command == "throw" and self.completed and not self.config.allow_repeat:
            self._emit_state("REJECTED_ALREADY_COMPLETED")
            self.result_callback("REJECTED:throw:already_completed")
            return
        try:
            self._move(self.config.safe_angle_deg)
            self.phase = command
            self.deadline = now + self.config.prepare_duration_sec
            self._emit_state(ServoState.PREPARING)
        except Exception as exc:
            self.fail(str(exc))
            return
        if command == "prepare":
            self.phase = "prepare_only"
        elif command == "reset":
            self.phase = "reset_only"

    def tick(self, now: float) -> None:
        if self.phase is None or now < self.deadline:
            return
        try:
            if self.phase in {"prepare_only", "reset_only"}:
                result_command = "prepare" if self.phase == "prepare_only" else "reset"
                self.phase = None
                self._emit_state(ServoState.READY)
                self.result_callback(f"SUCCESS:{result_command}")
            elif self.phase == "throw":
                self._move(self.config.release_angle_deg)
                self._emit_state(ServoState.RELEASING)
                self.phase = "release_wait"
                self.deadline = now + self.config.release_duration_sec
            elif self.phase == "release_wait":
                if self.config.return_after_release:
                    self._move(self.config.safe_angle_deg)
                    self._emit_state(ServoState.RETURNING)
                    self.phase = "return_wait"
                    self.deadline = now + self.config.return_duration_sec
                else:
                    self._finish_throw()
            elif self.phase == "return_wait":
                self._finish_throw()
        except Exception as exc:
            self.fail(str(exc))

    def _finish_throw(self) -> None:
        if self.config.disable_pwm_after_action:
            self.driver.stop()
        self.completed = True
        self.phase = None
        self._emit_state(ServoState.COMPLETED)
        self.result_callback("SUCCESS:throw")

    def fail(self, reason: str) -> None:
        self.phase = None
        self._emit_state(f"ERROR:{reason}")
        self.result_callback(f"FAILED:throw:{reason}")


class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")
        self._declare_parameters()
        config = self._read_config()
        errors = validate_config(config)
        if errors:
            raise ValueError("invalid servo parameters: " + "; ".join(errors))

        command_topic = self.get_parameter("command_topic").value
        status_topic = self.get_parameter("status_topic").value
        result_topic = self.get_parameter("result_topic").value
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
        self.status_pub = self.create_publisher(String, status_topic, status_qos)
        self.result_pub = self.create_publisher(String, result_topic, reliable_qos)
        self.command_sub = self.create_subscription(String, command_topic, self._on_command, reliable_qos)
        try:
            self.driver = ServoDriver(config.gpio_pin, config.dry_run, self.get_logger().info)
            if config.dry_run:
                self.get_logger().warning("DRY-RUN，不会驱动真实舵机")
            self.machine = ServoActionMachine(
                config, self.driver, self._publish_status, self._publish_result)
        except Exception:
            self._publish_status("ERROR:pigpio initialization failed")
            raise
        self.timer = self.create_timer(0.02, self._on_timer)

    def _declare_parameters(self) -> None:
        defaults = ServoConfig()
        for name, value in defaults.__dict__.items():
            self.declare_parameter(name, value)
        self.declare_parameter("command_topic", "/servo_command")
        self.declare_parameter("status_topic", "/servo/status")
        self.declare_parameter("result_topic", "/servo/result")

    def _read_config(self) -> ServoConfig:
        values = {name: self.get_parameter(name).value for name in ServoConfig.__dataclass_fields__}
        return ServoConfig(**values)

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self.status_pub.publish(message)

    def _publish_result(self, result: str) -> None:
        message = String()
        message.data = result
        self.result_pub.publish(message)

    def _on_command(self, message: String) -> None:
        self.machine.command(message.data, self.get_clock().now().nanoseconds / 1e9)

    def _on_timer(self) -> None:
        self.machine.tick(self.get_clock().now().nanoseconds / 1e9)

    def destroy_node(self):
        timer = getattr(self, "timer", None)
        if timer is not None:
            timer.cancel()
        driver = getattr(self, "driver", None)
        if driver is not None:
            try:
                driver.stop()
            except Exception as exc:
                self.get_logger().error(f"servo cleanup failed: {exc}")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoNode()
        rclpy.spin(node)
    except (ValueError, RuntimeError) as exc:
        print(f"servo_node not started: {exc}")
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
