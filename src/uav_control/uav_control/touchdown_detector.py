"""Conservative continuous touchdown detection without extra PX4 topics."""


class TouchdownDetector:
    def __init__(self, confirm_seconds=1.0, max_vz=0.12, max_tilt=0.18):
        self.confirm_seconds = float(confirm_seconds)
        self.max_vz = float(max_vz)
        self.max_tilt = float(max_tilt)
        self.candidate_since = None
        self.confirmed = False

    def update(self, now, sensor_contact, kinematic_contact, vz, roll, pitch):
        stable = abs(vz) <= self.max_vz and abs(
            roll) <= self.max_tilt and abs(pitch) <= self.max_tilt
        candidate = stable and (bool(sensor_contact) or bool(kinematic_contact))
        if not candidate:
            self.candidate_since = None
            self.confirmed = False
        elif self.candidate_since is None:
            self.candidate_since = float(now)
        elif now - self.candidate_since >= self.confirm_seconds:
            self.confirmed = True
        return self.confirmed
