from servo_control.servo_node import ServoActionMachine, ServoConfig, ServoState


class FakeDriver:
    def __init__(self):
        self.pulses = []
        self.stop_count = 0

    def set_pulsewidth(self, pulsewidth):
        self.pulses.append(pulsewidth)

    def stop(self):
        self.stop_count += 1


def make_machine(allow_repeat=False):
    states = []
    results = []
    driver = FakeDriver()
    machine = ServoActionMachine(
        ServoConfig(prepare_duration_sec=1.0, release_duration_sec=2.0,
                    return_duration_sec=3.0, allow_repeat=allow_repeat),
        driver, states.append, results.append)
    return machine, driver, states, results


def test_throw_follows_non_blocking_sequence_and_completes():
    machine, driver, states, results = make_machine()
    machine.command("throw", 0.0)
    machine.tick(1.0)
    machine.tick(3.0)
    machine.tick(6.0)
    assert states[-4:] == [ServoState.PREPARING.value, ServoState.RELEASING.value,
                            ServoState.RETURNING.value, ServoState.COMPLETED.value]
    assert results[-1] == "SUCCESS:throw"
    assert driver.pulses == [1000, 1500, 1000]


def test_duplicate_throw_is_rejected_while_busy_and_after_completion():
    machine, _, _, results = make_machine()
    machine.command("throw", 0.0)
    machine.command("throw", 0.1)
    assert results[-1] == "REJECTED:throw:busy"
    machine.tick(1.0)
    machine.tick(3.0)
    machine.tick(6.0)
    machine.command("throw", 7.0)
    assert results[-1] == "REJECTED:throw:already_completed"


def test_unknown_and_reset_do_not_clear_completion_and_stop_stops_pwm():
    machine, driver, _, results = make_machine()
    machine.command("unknown", 0.0)
    assert not driver.pulses
    machine.command("throw", 0.0)
    machine.tick(1.0)
    machine.tick(3.0)
    machine.tick(6.0)
    machine.command("reset", 7.0)
    machine.tick(8.0)
    assert machine.completed
    assert results[-1] == "SUCCESS:reset"
    machine.command("stop", 9.0)
    assert driver.stop_count >= 2
    assert results[-1] == "SUCCESS:stop"


def test_pwm_exception_enters_error_and_reports_failure():
    class FailingDriver(FakeDriver):
        def set_pulsewidth(self, pulsewidth):
            raise RuntimeError("pwm failed")

    states = []
    results = []
    machine = ServoActionMachine(ServoConfig(), FailingDriver(), states.append, results.append)
    machine.command("throw", 0.0)
    assert states[-1] == "ERROR:pwm failed"
    assert results[-1] == "FAILED:throw:pwm failed"
