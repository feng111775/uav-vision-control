from servo_control.servo_node import ServoConfig, ServoDriver, validate_config


class DisconnectedPi:
    connected = False


class PigpioModule:
    @staticmethod
    def pi():
        return DisconnectedPi()


class TrackingPi:
    connected = True

    def __init__(self):
        self.pulses = []
        self.stopped = False

    def set_servo_pulsewidth(self, pin, pulse):
        self.pulses.append((pin, pulse))
        return 0

    def stop(self):
        self.stopped = True


def test_invalid_config_is_rejected_before_hardware():
    errors = validate_config(ServoConfig(gpio_pin=-1, min_pulse_us=2000, max_pulse_us=1000))
    assert errors


def test_dry_run_does_not_import_or_connect_pigpio():
    driver = ServoDriver(18, dry_run=True)
    driver.set_pulsewidth(1500)
    driver.stop()


def test_pigpio_connection_failure_is_rejected():
    try:
        ServoDriver(18, dry_run=False, pigpio_module=PigpioModule)
    except RuntimeError as exc:
        assert "connection failed" in str(exc)
    else:
        raise AssertionError("disconnected pigpio must be rejected")


def test_cleanup_stops_pwm_with_zero_and_closes_connection():
    pi = TrackingPi()
    module = type("Module", (), {"pi": staticmethod(lambda: pi)})
    driver = ServoDriver(18, dry_run=False, pigpio_module=module)
    driver.stop()
    assert pi.pulses == [(18, 0)]
    assert pi.stopped
