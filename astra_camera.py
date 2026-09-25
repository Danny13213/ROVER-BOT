import numpy as np
import cv2

from primesense import openni2
from primesense import _openni2 as c_api


# ============================================================
# CAMERA SETTINGS
# ============================================================

WIDTH = 640
HEIGHT = 480
FPS = 30

# Astra Pro RGB camera
COLOR_INDEX = 0

# Private Orbbec OpenNI2 v2.3 runtime
_OPENNI2_LIB_PATH = "/opt/openni2-orbbec"


# ============================================================
# ASTRA CAMERA
# ============================================================

class AstraCamera:

    def __init__(self, width=WIDTH, height=HEIGHT, fps=FPS):

        self.width = width
        self.height = height
        self.fps = fps

        self._cap = None
        self._dev = None
        self._depth = None

        # ====================================================
        # RGB CAMERA
        # ====================================================

        print("Starting Astra RGB camera...")

        self._cap = cv2.VideoCapture(
            COLOR_INDEX,
            cv2.CAP_V4L2
        )

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Could not open RGB camera "
                f"(V4L2 index {COLOR_INDEX})"
            )

        # Force MJPG.
        # This is the mode that successfully returned frames
        # during the standalone Astra RGB test.
        self._cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        self._cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            width
        )

        self._cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            height
        )

        self._cap.set(
            cv2.CAP_PROP_FPS,
            fps
        )

        # Read back actual settings
        actual_width = int(
            self._cap.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        actual_height = int(
            self._cap.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        actual_fps = self._cap.get(
            cv2.CAP_PROP_FPS
        )

        print(
            f"RGB camera started: "
            f"{actual_width}x{actual_height} "
            f"@ {actual_fps:.1f} FPS"
        )

        # ====================================================
        # OPENNI2 DEPTH CAMERA
        # ====================================================

        print("Starting Astra depth camera...")

        try:

            openni2.initialize(
                _OPENNI2_LIB_PATH
            )

            self._dev = (
                openni2.Device.open_any()
            )

            self._depth = (
                self._dev.create_depth_stream()
            )

            mode = c_api.OniVideoMode(

                pixelFormat=(
                    c_api.OniPixelFormat.
                    ONI_PIXEL_FORMAT_DEPTH_1_MM
                ),

                resolutionX=width,
                resolutionY=height,
                fps=fps
            )

            self._depth.set_video_mode(
                mode
            )

            self._depth.start()

            print(
                f"Depth camera started: "
                f"{width}x{height} "
                f"@ {fps} FPS"
            )

        except Exception as e:

            # Make sure RGB is released if
            # OpenNI2 initialization fails.
            if self._cap is not None:
                self._cap.release()

            raise RuntimeError(
                f"DEPTH INITIALIZATION ERROR: {e}"
            )


    # ========================================================
    # READ CAMERA
    # ========================================================

    def read(self):
        """
        Returns:

            color_bgr:
                uint8 HxWx3 BGR image

            depth_mm:
                uint16 HxW depth image

        Depth value of 0 means invalid depth.
        """

        # ====================================================
        # READ RGB
        # ====================================================

        try:

            ret, color_bgr = (
                self._cap.read()
            )

        except Exception as e:

            raise RuntimeError(
                f"RGB READ ERROR: {e}"
            )

        if not ret:

            raise RuntimeError(
                "RGB ERROR: "
                "V4L2 returned ret=False"
            )

        if color_bgr is None:

            raise RuntimeError(
                "RGB ERROR: "
                "V4L2 returned an empty frame"
            )

        # Resize if the camera did not return
        # exactly 640x480.
        if (
            color_bgr.shape[1] != self.width
            or
            color_bgr.shape[0] != self.height
        ):

            color_bgr = cv2.resize(
                color_bgr,
                (
                    self.width,
                    self.height
                )
            )

        # ====================================================
        # READ DEPTH
        # ====================================================

        try:

            depth_frame = (
                self._depth.read_frame()
            )

        except Exception as e:

            raise RuntimeError(
                f"DEPTH READ ERROR: {e}"
            )

        if depth_frame is None:

            raise RuntimeError(
                "DEPTH ERROR: "
                "OpenNI2 returned no frame"
            )

        # ====================================================
        # CONVERT DEPTH FRAME TO NUMPY
        # ====================================================

        try:

            depth_mm = np.frombuffer(
                depth_frame.get_buffer_as_uint16(),
                dtype=np.uint16
            )

            expected_pixels = (
                self.width
                * self.height
            )

            if depth_mm.size != expected_pixels:

                raise RuntimeError(
                    f"Expected "
                    f"{expected_pixels} "
                    f"depth pixels, "
                    f"received "
                    f"{depth_mm.size}"
                )

            depth_mm = depth_mm.reshape(
                self.height,
                self.width
            ).copy()

        except Exception as e:

            raise RuntimeError(
                f"DEPTH CONVERSION ERROR: {e}"
            )

        # ====================================================
        # RETURN RGB + DEPTH
        # ====================================================

        return color_bgr, depth_mm


    # ========================================================
    # CLEANUP
    # ========================================================

    def release(self):

        print("Releasing Astra camera...")

        # Release RGB FIRST.
        #
        # Your OpenNI2 v2.3 plugin has previously caused
        # teardown problems, so release /dev/video0 before
        # touching OpenNI2.

        try:

            if self._cap is not None:

                self._cap.release()

                self._cap = None

        except Exception as e:

            print(
                "RGB release warning:",
                e
            )

        # Stop depth stream

        try:

            if self._depth is not None:

                self._depth.stop()

                self._depth = None

        except Exception as e:

            print(
                "Depth stop warning:",
                e
            )

        # Close OpenNI2 device

        try:

            if self._dev is not None:

                self._dev.close()

                self._dev = None

        except Exception as e:

            print(
                "OpenNI2 device close warning:",
                e
            )

        # Unload OpenNI2

        try:

            openni2.unload()

        except Exception as e:

            print(
                "OpenNI2 unload warning:",
                e
            )

        print("Astra camera released.")
