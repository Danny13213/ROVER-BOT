import numpy as np
import cv2
from primesense import openni2
from primesense import _openni2 as c_api

WIDTH = 640
HEIGHT = 480
FPS = 30

# The Astra Pro is two USB devices: depth (2bc5:0403) via OpenNI2, and a
# separate UVC color camera (2bc5:0501) at /dev/video0. They are NOT a single
# OpenNI2 device, so depth-to-color hardware registration is not available;
# bbox-to-depth alignment relies on the cameras being co-axial enough that
# median depth inside a YOLO bbox is meaningful.
# Numeric V4L2 index (not /dev/video0 string): OpenCV's V4L2 backend cannot
# capture by name on this platform, and the FFmpeg backend opens the device
# but silently returns black frames due to a missing VIDIOC_G_INPUT ioctl
# on the Astra Pro UVC firmware.
COLOR_INDEX = 0

# Private Orbbec OpenNI2 v2.3 redist (the plugin needs its matching runtime;
# the Debian system libOpenNI2.so is v2.2 and segfaults loading the v2.3 plugin).
_OPENNI2_LIB_PATH = "/opt/openni2-orbbec"


class AstraCamera:
    def __init__(self, width=WIDTH, height=HEIGHT, fps=FPS):
        self.width = width
        self.height = height
        self.fps = fps

        self._cap = cv2.VideoCapture(COLOR_INDEX, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open RGB camera (V4L2 index {COLOR_INDEX})")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        openni2.initialize(_OPENNI2_LIB_PATH)
        self._dev = openni2.Device.open_any()
        self._depth = self._dev.create_depth_stream()
        self._depth.set_video_mode(c_api.OniVideoMode(
            pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,
            resolutionX=width, resolutionY=height, fps=fps,
        ))
        self._depth.start()

    def read(self):
        """Returns (color_bgr uint8 HxWx3, depth_mm uint16 HxW). depth 0 = invalid."""
        ret, color_bgr = self._cap.read()
        if not ret:
            raise RuntimeError("Failed to grab RGB frame from V4L2 device")
        if color_bgr.shape[1] != self.width or color_bgr.shape[0] != self.height:
            color_bgr = cv2.resize(color_bgr, (self.width, self.height))

        df = self._depth.read_frame()
        depth_mm = np.frombuffer(df.get_buffer_as_uint16(), dtype=np.uint16)
        depth_mm = depth_mm.reshape(self.height, self.width).copy()
        return color_bgr, depth_mm

    def release(self):
        # Release V4L2 capture FIRST. The v2.3 OpenNI2 plugin emits a glibc
        # "corrupted size" abort() in its destructor on this Jetson, which
        # kills the whole process and skips any later cleanup — so anything
        # that must run (like releasing /dev/video0 so the next launch can
        # open it) has to happen before the OpenNI2 teardown.
        try:
            self._cap.release()
        except Exception:
            pass
        try:
            self._depth.stop()
        except Exception:
            pass
        try:
            self._dev.close()
        except Exception:
            pass
        try:
            openni2.unload()
        except Exception:
            pass
