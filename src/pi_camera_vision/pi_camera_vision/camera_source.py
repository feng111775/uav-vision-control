"""Camera acquisition backends for desktop development and Raspberry Pi."""

from pathlib import Path

import cv2


class OpenCVSource:
    """Read an image, video file, or USB camera through OpenCV."""

    def __init__(self, source_type, source='', device=0, loop=False,
                 width=640, height=480, fps=15.0):
        if source_type not in ('image', 'video', 'usb'):
            raise ValueError('OpenCV source_type must be image, video, or usb')
        self.source_type = source_type
        self.loop = bool(loop)
        self.image = None
        self.capture = None
        if source_type == 'image':
            path = Path(source).expanduser()
            self.image = cv2.imread(str(path))
            if self.image is None:
                raise RuntimeError('cannot read image: %s' % path)
        else:
            target = int(device) if source_type == 'usb' else str(
                Path(source).expanduser())
            self.capture = cv2.VideoCapture(target)
            if not self.capture.isOpened():
                self.capture.release()
                raise RuntimeError('cannot open %s source: %s' % (
                    source_type, target))
            if source_type == 'usb':
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
                self.capture.set(cv2.CAP_PROP_FPS, float(fps))

    def read(self):
        """Return (ok, BGR frame); image sources return one frame."""
        if self.source_type == 'image':
            if self.image is None:
                return False, None
            frame, self.image = self.image, None
            return True, frame.copy()
        ok, frame = self.capture.read()
        if not ok and self.source_type == 'video' and self.loop:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
        return bool(ok), frame

    def close(self):
        """Release the OpenCV capture if one was opened."""
        if self.capture is not None:
            self.capture.release()


class Picamera2Source:
    """Use Picamera2 only when it is installed on a Raspberry Pi."""

    def __init__(self, width=640, height=480, fps=15.0):
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError(
                'Picamera2 is unavailable; install it only on Raspberry Pi'
            ) from error
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={'size': (int(width), int(height)), 'format': 'BGR888'})
        self.camera.configure(config)
        self.camera.set_controls({'FrameRate': float(fps)})
        self.camera.start()

    def read(self):
        """Capture one BGR888 frame."""
        frame = self.camera.capture_array('main')
        return frame is not None, frame

    def close(self):
        """Stop the Raspberry Pi camera."""
        self.camera.stop()
        self.camera.close()


def create_source(source_type, source='', device=0, loop=False,
                  width=640, height=480, fps=15.0):
    """Create the selected acquisition backend."""
    if source_type == 'picamera2':
        return Picamera2Source(width=width, height=height, fps=fps)
    return OpenCVSource(
        source_type=source_type, source=source, device=device, loop=loop,
        width=width, height=height, fps=fps)
