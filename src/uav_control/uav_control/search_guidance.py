"""Bounded heading-locked search guidance for SEARCH_CAR."""

from __future__ import annotations

import math


class SearchGuidance:
    """Finite waypoint search centred on the recorded takeoff home point."""

    def __init__(self, speed_mps=0.18, forward_offset_m=0.50,
                 forward_span_m=0.40, lateral_extent_m=0.25,
                 arrival_tolerance_m=0.08, waypoint_timeout_s=6.0,
                 max_radius_m=1.0):
        values = (speed_mps, forward_offset_m, forward_span_m,
                  lateral_extent_m, arrival_tolerance_m,
                  waypoint_timeout_s, max_radius_m)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('search parameters must be finite')
        if not 0.0 < float(speed_mps) <= 0.18:
            raise ValueError('search speed must be in (0, 0.18] m/s')
        if min(float(forward_offset_m), float(forward_span_m),
               float(lateral_extent_m), float(arrival_tolerance_m),
               float(waypoint_timeout_s), float(max_radius_m)) <= 0.0:
            raise ValueError('search distances and timeout must be positive')
        self.speed = float(speed_mps)
        self.forward_offset = float(forward_offset_m)
        self.forward_span = float(forward_span_m)
        self.lateral_extent = float(lateral_extent_m)
        self.arrival_tolerance = float(arrival_tolerance_m)
        self.waypoint_timeout = float(waypoint_timeout_s)
        self.max_radius = float(max_radius_m)
        self.reset()

    def reset(self):
        self.home = None
        self.heading = None
        self.waypoints = ()
        self.index = 0
        self.started_at = None
        self.waypoint_started_at = None

    @staticmethod
    def _rotate(forward, right, heading):
        return (forward * math.cos(heading) - right * math.sin(heading),
                forward * math.sin(heading) + right * math.cos(heading))

    def start(self, home_x, home_y, heading, now):
        values = (home_x, home_y, heading, now)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('search start values must be finite')
        self.home = (float(home_x), float(home_y))
        self.heading = float(heading)
        local_waypoints = (
            (self.forward_offset, 0.0),
            (self.forward_offset + self.forward_span,
             self.lateral_extent),
            (self.forward_offset + self.forward_span,
             -self.lateral_extent),
        )
        self.waypoints = tuple(
            self._rotate(forward, right, self.heading)
            for forward, right in local_waypoints)
        if any(math.hypot(*point) > self.max_radius + 1e-9
               for point in self.waypoints):
            raise ValueError('search waypoint exceeds max radius')
        self.index = 0
        self.started_at = float(now)
        self.waypoint_started_at = float(now)

    @property
    def complete(self):
        return bool(self.waypoints) and self.index >= len(self.waypoints)

    def velocity(self, position, now, paused=False):
        """Return a bounded NED (north, east) velocity, or zero to hover."""
        if self.home is None or self.complete or paused:
            return (0.0, 0.0)
        if position is None or len(position) < 2:
            return (0.0, 0.0)
        now = float(now)
        current = (float(position[0]), float(position[1]))
        if not all(math.isfinite(value) for value in (*current, now)):
            return (0.0, 0.0)
        waypoint = (self.home[0] + self.waypoints[self.index][0],
                    self.home[1] + self.waypoints[self.index][1])
        dx = waypoint[0] - current[0]
        dy = waypoint[1] - current[1]
        distance = math.hypot(dx, dy)
        if distance <= self.arrival_tolerance or (
                now - self.waypoint_started_at >= self.waypoint_timeout):
            self.index += 1
            self.waypoint_started_at = now
            if self.complete:
                return (0.0, 0.0)
            waypoint = (self.home[0] + self.waypoints[self.index][0],
                        self.home[1] + self.waypoints[self.index][1])
            dx = waypoint[0] - current[0]
            dy = waypoint[1] - current[1]
            distance = math.hypot(dx, dy)
        if distance <= 1e-9:
            return (0.0, 0.0)
        scale = min(self.speed / distance, 1.0)
        return (dx * scale, dy * scale)
