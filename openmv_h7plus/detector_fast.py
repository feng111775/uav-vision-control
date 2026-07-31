"""Blob-first detector: expensive circular/cross validation is periodic only."""
from detector import DTaskDetector, _invalid
from tracker import TrackState
from config import SEARCH_FULL_VERIFY_INTERVAL, TRACK_FULL_VERIFY_INTERVAL, ROI_SCALE, ROI_MAX_LOST_FRAMES

class FastV2Detector(DTaskDetector):
    def __init__(self, enable_timing=False):
        super().__init__(enable_timing); self.track=TrackState(); self.mode='SEARCH'
    def detect(self,image,mission_mode='SEARCH'):
        self.mode=mission_mode
        roi=self.track.roi(image,ROI_SCALE) if mission_mode in ('FOLLOW','DROP_ALIGN') or self.track.last else None
        # Blob filtering runs every real image. Full Hough verification runs only
        # when candidate geometry exists and periodically while tracking.
        candidate=None
        search_roi=roi or (0,0,image.width(),image.height())
        started = self.timing.begin()
        regions=self._search_regions(image,search_roi)
        self.timing.end('candidate_search', started)
        verify_every=TRACK_FULL_VERIFY_INTERVAL if roi else SEARCH_FULL_VERIFY_INTERVAL
        force=(self.track.frame % verify_every)==0
        if regions and (force or roi is not None):
            for region in regions:
                started = self.timing.begin()
                pairs=self._circle_pairs(image,region)
                self.timing.end('strong_verify', started)
                if pairs:
                    started = self.timing.begin()
                    candidate=self._detect_roi(image,region)
                    self.timing.end('roi_detect', started)
                    break
        if candidate is None:
            self.track.update(None)
            if self.track.lost>ROI_MAX_LOST_FRAMES: self.track.last=None; self.mode='SEARCH'
            return _invalid('LOST')
        self.track.update(candidate); self.last=candidate
        return candidate
