# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Pure health aggregation used by ROS and unit tests."""
from dataclasses import dataclass, field
from typing import Callable

OK = "OK"
WARN = "WARN"
ERROR = "ERROR"
STALE = "STALE"
DISABLED = "DISABLED"
UNKNOWN = "UNKNOWN"


@dataclass
class WatchedSource:  # noqa: D101
    timeout: float
    enabled: bool = True
    last_seen: float | None = None
    detail: str = ""

    def state(self, now: float) -> str:  # noqa: D102
        if not self.enabled:
            return DISABLED
        if self.last_seen is None or now - self.last_seen > self.timeout:
            return STALE
        return OK


@dataclass
class HealthModel:  # noqa: D101
    sources: dict[str, WatchedSource] = field(default_factory=dict)

    def observe(self, name: str, now: float, detail: str = "") -> None:  # noqa: D102
        if name in self.sources:
            self.sources[name].last_seen = now
            self.sources[name].detail = detail

    def snapshot(self, now: float) -> dict[str, str]:  # noqa: D102
        return {name: source.state(now) for name, source in self.sources.items()}


def read_cpu_temperature(path: str = "/sys/class/thermal/thermal_zone0/temp") -> tuple[str, float | None]:  # noqa: D103
    try:
        with open(path, encoding="ascii") as stream:
            return OK, float(stream.read().strip()) / 1000.0
    except (OSError, ValueError):
        return UNKNOWN, None


def classify_disk(free_fraction: float, warn_fraction: float = 0.15) -> str:  # noqa: D103
    return WARN if free_fraction < warn_fraction else OK


def read_memory_usage(path: str = '/proc/meminfo') -> tuple[str, float | None]:
    """Return used-memory fraction from Linux meminfo."""
    try:
        with open(path, encoding='ascii') as stream:
            values = {
                line.split(':', 1)[0]: int(line.split()[1])
                for line in stream if ':' in line
            }
        return OK, 1.0 - values['MemAvailable'] / values['MemTotal']
    except (OSError, ValueError, KeyError, ZeroDivisionError):
        return UNKNOWN, None


def read_clock_sync(path: str = '/run/systemd/timesync/synchronized') -> str:
    """Read systemd's standard synchronization marker without subprocesses."""
    try:
        with open(path, encoding='ascii') as stream:
            return OK if stream.read().strip().lower() == 'yes' else WARN
    except OSError:
        return UNKNOWN


def safe_resource(reader: Callable[[], float]) -> tuple[str, float | None]:  # noqa: D103
    try:
        return OK, float(reader())
    except (OSError, ValueError):
        return UNKNOWN, None
