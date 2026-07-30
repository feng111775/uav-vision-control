"""Camera-error to body FLU and local NED velocity guidance."""

import math


class VisualGuidance:
    def __init__(self, kp_forward=0.3, kp_left=0.3, deadband_x=0.04,
                 deadband_y=0.04, max_speed=0.5, min_confidence=60.0,
                 max_age_ms=250.0, camera_x_sign=1.0, camera_y_sign=-1.0,
                 max_error_jump=0.8, swap_axes=False, error_scale=1.0,
                 camera_mount_yaw_rad=0.0):
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
        parameters = (
            self.kp_forward, self.kp_left, self.deadband_x, self.deadband_y,
            self.max_speed, self.min_confidence, self.max_age_ms,
            self.camera_x_sign, self.camera_y_sign, self.max_error_jump)
        parameters = parameters + (
            self.error_scale, self.camera_mount_yaw_rad)
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
        self.last_error = None

    @staticmethod
    def flu_to_ned(forward, left, heading):
        values = (float(forward), float(left), float(heading))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('FLU velocity and heading must be finite')
        forward, left, heading = values
        right = -left
        return (math.cos(heading) * forward - math.sin(heading) * right,
                math.sin(heading) * forward + math.cos(heading) * right)

    def velocity(self, valid, error_x, error_y, confidence, age_ms, heading):
        values = (float(error_x), float(error_y), float(confidence),
                  float(age_ms), float(heading))
        if not all(math.isfinite(value) for value in values):
            self.last_error = None
            return 0.0, 0.0
        error_x, error_y, confidence, age_ms, heading = values
        error_x /= self.error_scale
        error_y /= self.error_scale
        if self.swap_axes:
            error_x, error_y = error_y, error_x
        if confidence < 0.0 or confidence > 100.0 or age_ms < 0.0:
            self.last_error = None
            return 0.0, 0.0
        if not valid or confidence < self.min_confidence or age_ms > self.max_age_ms:
            self.last_error = None
            return 0.0, 0.0
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
        return north, east
