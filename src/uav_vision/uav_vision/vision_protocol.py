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
        try:
            sequence = int(f[1]); ticks = int(f[2]); processing_us = int(f[3])
            valid = int(f[5]); values = [float(v) for v in f[6:12]]
        except (TypeError, ValueError) as error:
            raise ValueError('malformed V2 numeric field') from error
        if sequence < 0 or sequence > 0xffffffff or ticks < 0 or ticks > 0xffffffff:
            raise ValueError('V2 sequence or capture ticks outside uint32')
        if processing_us < 0 or valid not in (0, 1) or not all(math.isfinite(v) for v in values):
            raise ValueError('malformed V2 values')
        return dict(version=2, sequence=sequence, ticks=ticks, processing_us=processing_us,
                    mode=f[4], valid=bool(valid), cx=values[0], cy=values[1],
                    outer=values[2], inner=values[3], angle=values[4], confidence=values[5])
    raise ValueError('unsupported or malformed target line')

class ClockMapper:
    """Conservative monotonic millisecond mapper: valid only after stable samples."""
    def __init__(self): self.last_ticks=None; self.wraps=0; self.offsets=[]; self.synced=False
    def reset(self): self.__init__()
    def update(self, ticks_ms, receive_ns):
        if self.last_ticks is not None and ticks_ms < self.last_ticks and self.last_ticks-ticks_ms > 100000:
            self.wraps += 1
        self.last_ticks=ticks_ms; extended=ticks_ms + self.wraps*(1 << 32)
        offset=receive_ns-extended*1000000; self.offsets.append(offset)
        self.offsets=self.offsets[-32:]
        self.synced=len(self.offsets)>=8 and max(self.offsets)-min(self.offsets)<50000000
        return (extended*1000000 + min(self.offsets), self.synced)


class SequenceTracker:
    """Classify uint32 frame sequences while suppressing duplicate publishes."""
    def __init__(self):
        self.last = None
        self.duplicate_count = 0
        self.dropped_count = 0
        self.out_of_order_count = 0
        self.restart_count = 0

    def reset(self):
        self.__init__()

    def accept(self, sequence):
        sequence = int(sequence) & 0xffffffff
        if self.last is None:
            self.last = sequence
            return True, 'new'
        delta = (sequence - self.last) & 0xffffffff
        if delta == 0:
            self.duplicate_count += 1
            return False, 'duplicate'
        if delta < 0x80000000:
            if delta > 1:
                self.dropped_count += delta - 1
            self.last = sequence
            return True, 'new'
        if sequence == 1 and self.last > 1:
            self.restart_count += 1
            self.last = sequence
            return True, 'restart'
        if self.last >= 0xf0000000 and sequence <= 0x0fffffff:
            self.last = sequence
            return True, 'wrap'
        self.out_of_order_count += 1
        return False, 'out_of_order'

class Confirmation:
    def __init__(self): self.count=0; self.last_sequence=None
    def ingest(self, item):
        new=item['sequence'] != self.last_sequence
        if new: self.last_sequence=item['sequence']
        valid = new and item['valid'] and item['confidence'] >= 60.0 and all(math.isfinite(item[k]) for k in ('cx','cy','outer','inner','angle'))
        self.count=self.count+1 if valid else 0
        return valid, self.count >= 3, self.count
