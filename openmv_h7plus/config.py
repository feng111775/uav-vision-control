"""Tunable safe defaults for H7 Plus / OV5640 QVGA."""
DETECTOR_BACKEND = 'fast_v2'       # set to legacy for proven Hough-only fallback
IMAGE_WIDTH, IMAGE_HEIGHT = 320, 240
MIN_CONFIDENCE, CONFIRM_FRAMES = 60, 3
SEARCH_FULL_VERIFY_INTERVAL = 8
TRACK_FULL_VERIFY_INTERVAL = 12
ROI_SCALE = 1.65
ROI_EXPAND_STEP, ROI_MAX_LOST_FRAMES = 1.35, 3
MIN_OUTER_DIAMETER, MAX_OUTER_DIAMETER = 24, 220
INNER_OUTER_RATIO_MIN, INNER_OUTER_RATIO_MAX = .52, .68
OUTPUT_PERIOD_MS = 0               # every snapshot is a new observation
TIMING_ENABLED = False
# One transport only; stdout is the CDC stream confirmed on /dev/ttyACM0.
PROTOCOL_TRANSPORT = 'stdout'
USB_VCP_ID = 0
DEBUG_DRAW = False
