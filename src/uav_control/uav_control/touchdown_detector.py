"""Conservative continuous touchdown detection without extra PX4 topics."""


class TouchdownDetector:
    def __init__(self, confirm_seconds=1.0, max_vz=0.12, max_tilt=0.18):
        self.confirm_seconds = float(confirm_seconds)
        self.max_vz = float(max_vz)
        self.max_tilt = float(max_tilt)
        self.candidate_since = None
        self.confirmed = False

    def candidate(self, sensor_contact, kinematic_contact, vz, roll, pitch):
        """Return the instantaneous conservative touchdown candidate."""
        stable = abs(vz) <= self.max_vz and abs(
            roll) <= self.max_tilt and abs(pitch) <= self.max_tilt
        return stable and (bool(sensor_contact) or bool(kinematic_contact))

    def update(self, now, sensor_contact, kinematic_contact, vz, roll, pitch):
        candidate = self.candidate(
            sensor_contact, kinematic_contact, vz, roll, pitch)
        if not candidate:
            self.candidate_since = None
            self.confirmed = False
        elif self.candidate_since is None:
            self.candidate_since = float(now)
        elif now - self.candidate_since >= self.confirm_seconds:
            self.confirmed = True
        return self.confirmed
