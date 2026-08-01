"""Camera-error to body FLU and local NED velocity guidance."""

import math


def map_camera_error(error_x, error_y, swap_axes=False, x_sign=1.0,
                     y_sign=1.0, target_x=0.0, target_y=0.0):
    """Map camera-normalized error into configurable controller axes."""
    values = (float(error_x), float(error_y), float(x_sign), float(y_sign),
              float(target_x), float(target_y))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('camera error mapping values must be finite')
    if values[2] not in (-1.0, 1.0) or values[3] not in (-1.0, 1.0):
        raise ValueError('camera error signs must be -1 or 1')
    mapped = (values[1], values[0]) if swap_axes else values[:2]
    return (values[2] * mapped[0] - values[4],
            values[3] * mapped[1] - values[5])


class VisualGuidance:
    def __init__(self, kp_forward=0.3, kp_left=0.3, deadband_x=0.04,
                 deadband_y=0.04, max_speed=0.5, min_confidence=60.0,
                 max_age_ms=250.0, camera_x_sign=1.0, camera_y_sign=-1.0,
                 max_error_jump=0.8, swap_axes=False, error_scale=1.0,
                 camera_mount_yaw_rad=0.0, max_acceleration=None):
        self.kp_forward = float(kp_forward)
        self.kp_left = float(kp_left)
        self.deadband_x = float(deadband_x)
        self.deadband_y = float(deadband_y)
        self.max_speed = float(max_speed)
        self.min_confidence = float(min_confidence)
        self.max_age_ms = float(max_age_ms)
        self.camera_x_sign = float(camera_x_sign)
        self.camera_y_sign = float(camera_y_sign)
        self.max_error_jump = float(max_error_jump)
        self.swap_axes = bool(swap_axes)
        self.error_scale = float(error_scale)
        self.camera_mount_yaw_rad = float(camera_mount_yaw_rad)
        self.max_acceleration = (None if max_acceleration is None else
                                 float(max_acceleration))
        parameters = (
            self.kp_forward, self.kp_left, self.deadband_x, self.deadband_y,
            self.max_speed, self.min_confidence, self.max_age_ms,
            self.camera_x_sign, self.camera_y_sign, self.max_error_jump)
        parameters = parameters + (
            self.error_scale, self.camera_mount_yaw_rad)
        if self.max_acceleration is not None:
            parameters = parameters + (self.max_acceleration,)
        if not all(math.isfinite(value) for value in parameters):
            raise ValueError('visual-guidance parameters must be finite')
        if min(self.kp_forward, self.kp_left, self.deadband_x,
               self.deadband_y, self.max_speed, self.max_age_ms,
               self.max_error_jump) < 0.0:
            raise ValueError('visual-guidance limits and gains cannot be negative')
        if not 0.0 <= self.min_confidence <= 100.0:
            raise ValueError('min_confidence must be in [0, 100]')
        if self.camera_x_sign not in (-1.0, 1.0) or self.camera_y_sign not in (-1.0, 1.0):
            raise ValueError('camera signs must be -1 or 1')
        if self.error_scale <= 0.0:
            raise ValueError('error_scale must be positive')
        if (self.max_acceleration is not None and
                self.max_acceleration <= 0.0):
            raise ValueError('max_acceleration must be positive')
        self.last_error = None
        self.last_velocity = (0.0, 0.0)

    @staticmethod
    def flu_to_ned(forward, left, heading):
        values = (float(forward), float(left), float(heading))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('FLU velocity and heading must be finite')
        forward, left, heading = values
        right = -left
        return (math.cos(heading) * forward - math.sin(heading) * right,
                math.sin(heading) * forward + math.cos(heading) * right)

    def velocity(self, valid, error_x, error_y, confidence, age_ms, heading,
                 dt=None):
        values = (float(error_x), float(error_y), float(confidence),
                  float(age_ms), float(heading))
        if not all(math.isfinite(value) for value in values):
            self.last_error = None
            return self._limited_velocity((0.0, 0.0), dt)
        error_x, error_y, confidence, age_ms, heading = values
        error_x /= self.error_scale
        error_y /= self.error_scale
        if self.swap_axes:
            error_x, error_y = error_y, error_x
        if confidence < 0.0 or confidence > 100.0 or age_ms < 0.0:
            self.last_error = None
            return self._limited_velocity((0.0, 0.0), dt)
        if not valid or confidence < self.min_confidence or age_ms > self.max_age_ms:
            self.last_error = None
            return self._limited_velocity((0.0, 0.0), dt)
        current = (float(error_x), float(error_y))
        if self.last_error and math.hypot(current[0] - self.last_error[0],
                                          current[1] - self.last_error[1]) > self.max_error_jump:
            self.last_error = current
            return 0.0, 0.0
        self.last_error = current
        left = 0.0 if abs(error_x) <= self.deadband_x else self.camera_x_sign * \
            self.kp_left * error_x
        forward = 0.0 if abs(error_y) <= self.deadband_y else self.camera_y_sign * \
            self.kp_forward * error_y
        cosine = math.cos(self.camera_mount_yaw_rad)
        sine = math.sin(self.camera_mount_yaw_rad)
        mounted_forward = cosine * forward - sine * left
        mounted_left = sine * forward + cosine * left
        north, east = self.flu_to_ned(
            mounted_forward, mounted_left, heading)
        magnitude = math.hypot(north, east)
        if magnitude > self.max_speed:
            scale = self.max_speed / magnitude
            north *= scale
            east *= scale
        return self._limited_velocity((north, east), dt)

    def _limited_velocity(self, target, dt):
        """Apply an optional acceleration limit to a finite N/E target."""
        target = (float(target[0]), float(target[1]))
        if not all(math.isfinite(value) for value in target):
            self.last_velocity = (0.0, 0.0)
            return self.last_velocity
        if self.max_acceleration is None or dt is None:
            self.last_velocity = target
            return target
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            self.last_velocity = (0.0, 0.0)
            return self.last_velocity
        dx = target[0] - self.last_velocity[0]
        dy = target[1] - self.last_velocity[1]
        distance = math.hypot(dx, dy)
        maximum_delta = self.max_acceleration * dt
        if distance > maximum_delta:
            scale = maximum_delta / distance
            target = (self.last_velocity[0] + dx * scale,
                      self.last_velocity[1] + dy * scale)
        self.last_velocity = target
        return target
