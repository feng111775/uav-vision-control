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

    def request(self, command, now):
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
        self.attempts += 1
        self.status = 'SENT'
        return True

    def acknowledge(self, command, result):
        if int(command) != self.command:
            return False
        self.status = ACK_NAMES.get(int(result), 'UNKNOWN_%d' % result)
        if result == 1:  # temporary rejection may be retried after timeout
            self.sent_at = None
        return True

    @property
    def failed(self):
        return self.status in ('DENIED', 'UNSUPPORTED', 'FAILED', 'CANCELLED',
                               'TIMEOUT')
