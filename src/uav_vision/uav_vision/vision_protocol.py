"""Protocol V1/V2 parsing and state logic; deliberately CPython-testable."""
import math

V2_PREFIX = 'D_TARGET_V2'
V1_PREFIX = 'D_TARGET'

def parse_target(line):
    f = line.strip().split(',')
    if len(f) == 8 and f[0] == V1_PREFIX:
        return dict(version=1, sequence=None, ticks=None, processing_us=None,
                    mode='SEARCH', valid=bool(int(f[1])), cx=float(f[2]), cy=float(f[3]),
                    outer=float(f[4]), inner=float(f[5]), angle=float(f[6]), confidence=float(f[7]))
    if len(f) == 13 and f[0] == V2_PREFIX:
        return dict(version=2, sequence=int(f[1]), ticks=int(f[2]), processing_us=int(f[3]),
                    mode=f[4], valid=bool(int(f[5])), cx=float(f[6]), cy=float(f[7]),
                    outer=float(f[8]), inner=float(f[9]), angle=float(f[10]), confidence=float(f[11]),
                    # field 12 reserved for forward-compatible detector tag
                    detector=f[12])
    # Current emitted V2 has twelve payload fields after prefix; accept it too.
    if len(f) == 12 and f[0] == V2_PREFIX:
        return dict(version=2, sequence=int(f[1]), ticks=int(f[2]), processing_us=int(f[3]),
                    mode=f[4], valid=bool(int(f[5])), cx=float(f[6]), cy=float(f[7]),
                    outer=float(f[8]), inner=float(f[9]), angle=float(f[10]), confidence=float(f[11]))
    raise ValueError('unsupported or malformed target line')

class ClockMapper:
    """Conservative monotonic millisecond mapper: valid only after stable samples."""
    def __init__(self): self.last_ticks=None; self.wraps=0; self.offsets=[]; self.synced=False
    def reset(self): self.__init__()
    def update(self, ticks_ms, receive_ns):
        if self.last_ticks is not None and ticks_ms < self.last_ticks and self.last_ticks-ticks_ms > 100000:
            self.wraps += 1
        self.last_ticks=ticks_ms; extended=ticks_ms + self.wraps*(1 << 30)
        offset=receive_ns-extended*1000000; self.offsets.append(offset)
        self.offsets=self.offsets[-32:]
        self.synced=len(self.offsets)>=8 and max(self.offsets)-min(self.offsets)<50000000
        return (extended*1000000 + min(self.offsets), self.synced)

class Confirmation:
    def __init__(self): self.count=0; self.last_sequence=None
    def ingest(self, item):
        new=item['sequence'] != self.last_sequence
        if new: self.last_sequence=item['sequence']
        valid = new and item['valid'] and item['confidence'] >= 60.0 and all(math.isfinite(item[k]) for k in ('cx','cy','outer','inner','angle'))
        self.count=self.count+1 if valid else 0
        return valid, self.count >= 3, self.count
