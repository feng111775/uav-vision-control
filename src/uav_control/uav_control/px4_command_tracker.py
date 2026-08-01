"""Finite retry and acknowledgement tracking for PX4 VehicleCommand."""


ACK_NAMES = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
             3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'}
TERMINAL_FAILURES = {2, 3, 4, 6}


class CommandTracker:
    def __init__(self, timeout=1.0, max_attempts=3):
        self.timeout = float(timeout)
        self.max_attempts = int(max_attempts)
        self.reset()

    def reset(self):
        self.command = None
        self.sent_at = None
        self.attempts = 0
        self.status = 'IDLE'
        self.first_sent_at = None
        self.generation = 0
        self.target_system = 1
        self.target_component = 1
        self.context = ''
        self.acknowledged_at = None

    def request(self, command, now, context='', target_system=1,
                target_component=1):
        if self.command != command:
            self.reset()
            self.command = int(command)
        if self.status in ('ACCEPTED', 'DENIED', 'UNSUPPORTED', 'FAILED',
                           'CANCELLED'):
            return False
        if self.sent_at is not None and now - self.sent_at < self.timeout:
            return False
        if self.attempts >= self.max_attempts:
            self.status = 'TIMEOUT'
            return False
        self.sent_at = float(now)
        if self.first_sent_at is None:
            self.first_sent_at = self.sent_at
            self.generation += 1
        self.context = str(context)
        self.target_system = int(target_system)
        self.target_component = int(target_component)
        self.attempts += 1
        self.status = 'SENT'
        return True

    def acknowledge(self, command, result, now=None, target_system=None,
                    target_component=None, context=None,
                    from_external=None):
        if int(command) != self.command or self.status != 'SENT':
            return False
        if now is None:
            # Compatibility for pure-logic callers. ROS adapters always pass
            # the actual receive time for stale-ACK rejection.
            now = self.sent_at
        if self.sent_at is None or float(now) < self.sent_at:
            return False
        if float(now) - self.sent_at > self.timeout:
            return False
        if (target_system is not None and int(target_system) not in
                (0, self.target_system)):
            return False
        if (target_component is not None and int(target_component) not in
                (0, self.target_component)):
            return False
        if context is not None and str(context) != self.context:
            return False
        if from_external is not None and not bool(from_external):
            return False
        self.status = ACK_NAMES.get(int(result), 'UNKNOWN_%d' % result)
        self.acknowledged_at = float(now)
        if result == 1:  # temporary rejection may be retried after timeout
            self.sent_at = None
        return True

    @property
    def failed(self):
        return self.status in ('DENIED', 'UNSUPPORTED', 'FAILED', 'CANCELLED',
                               'TIMEOUT')
