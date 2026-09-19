import sys
import os
import time

import cv2
import numpy as np

from flask import Flask, Response


# ============================================================
# IMPORT ROVER CAMERA / NAVIGATION MODULES
# ============================================================

sys.path.append("scripts")

from astra_camera import AstraCamera

from nav_logic import (
    decide,
    analyze_depth_zones,
    BAND_TOP_FRAC,
    BAND_BOT_FRAC
)


# ============================================================
# FLASK
# ============================================================

app = Flask(__name__)


# ============================================================
# CAMERA
# ============================================================

cam = AstraCamera()


# ============================================================
# NAVIGATION OUTPUT FILE
# ============================================================

NAV_FILE = "/tmp/rover_nav.txt"
NAV_TEMP_FILE = "/tmp/rover_nav.txt.tmp"


# ============================================================
# DISTANCE DISPLAY
# ============================================================

def distance_text(distance):

    if distance == float("inf"):

        return "BLOCKED / UNKNOWN"

    return f"{distance:.2f} m"


# ============================================================
# WRITE NAVIGATION DATA
# ============================================================

def publish_navigation(
    left_m,
    center_m,
    right_m,
    decision
):

    try:

        timestamp = time.time()

        with open(
            NAV_TEMP_FILE,
            "w"
        ) as f:

            f.write(
                f"{timestamp} "
                f"{left_m} "
                f"{center_m} "
                f"{right_m} "
                f"{decision['throttle']} "
                f"{decision['steer']} "
                f"{decision['status']}\n"
            )

        # Atomic replacement.
        # rover_control.cpp will never read
        # a partially written file.

        os.replace(
            NAV_TEMP_FILE,
            NAV_FILE
        )

    except Exception as e:

        print(
            "Navigation file error:",
            e
        )


# ============================================================
# FRAME GENERATOR
# ============================================================

def generate_frames():

    while True:

        try:

            color, depth_mm = cam.read()

        except Exception as e:

            print(
                "Camera error:",
                e
            )

            continue


        # ----------------------------------------------------
        # DEPTH ANALYSIS
        # ----------------------------------------------------

        left_m, center_m, right_m = (
            analyze_depth_zones(
                depth_mm
            )
        )


        # ----------------------------------------------------
        # NAVIGATION DECISION
        # ----------------------------------------------------

        decision = decide(
            (
                left_m,
                center_m,
                right_m
            )
        )

        status = decision["status"]


        # ----------------------------------------------------
        # PUBLISH DATA FOR C++ MOTOR CONTROLLER
        # ----------------------------------------------------

        publish_navigation(
            left_m,
            center_m,
            right_m,
            decision
        )


        # ----------------------------------------------------
        # DRAW NAVIGATION ZONES
        # ----------------------------------------------------

        h, w = color.shape[:2]

        y0 = int(
            h * BAND_TOP_FRAC
        )

        y1 = int(
            h * BAND_BOT_FRAC
        )

        zone_width = w // 3


        # Main navigation area

        cv2.rectangle(
            color,
            (0, y0),
            (w - 1, y1),
            (255, 255, 255),
            2
        )


        # Left / center divider

        cv2.line(
            color,
            (zone_width, y0),
            (zone_width, y1),
            (255, 255, 255),
            2
        )


        # Center / right divider

        cv2.line(
            color,
            (
                zone_width * 2,
                y0
            ),
            (
                zone_width * 2,
                y1
            ),
            (255, 255, 255),
            2
        )


        # ----------------------------------------------------
        # CENTER CROSSHAIR
        # ----------------------------------------------------

        center_x = w // 2
        center_y = h // 2

        crosshair_size = 15

        cv2.line(
            color,
            (
                center_x
                - crosshair_size,
                center_y
            ),
            (
                center_x
                + crosshair_size,
                center_y
            ),
            (255, 255, 255),
            2
        )

        cv2.line(
            color,
            (
                center_x,
                center_y
                - crosshair_size
            ),
            (
                center_x,
                center_y
                + crosshair_size
            ),
            (255, 255, 255),
            2
        )


        # ----------------------------------------------------
        # LEFT DISTANCE
        # ----------------------------------------------------

        cv2.putText(
            color,
            "LEFT",
            (
                20,
                y0 + 25
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            color,
            distance_text(
                left_m
            ),
            (
                20,
                y0 + 50
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )


        # ----------------------------------------------------
        # CENTER DISTANCE
        # ----------------------------------------------------

        cv2.putText(
            color,
            "CENTER",
            (
                zone_width + 20,
                y0 + 25
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            color,
            distance_text(
                center_m
            ),
            (
                zone_width + 20,
                y0 + 50
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )


        # ----------------------------------------------------
        # RIGHT DISTANCE
        # ----------------------------------------------------

        cv2.putText(
            color,
            "RIGHT",
            (
                zone_width * 2 + 20,
                y0 + 25
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            color,
            distance_text(
                right_m
            ),
            (
                zone_width * 2 + 20,
                y0 + 50
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )


        # ----------------------------------------------------
        # STATUS COLOR
        # ----------------------------------------------------

        if status == "STOP":

            status_color = (
                0,
                0,
                255
            )

        elif status == "AVOID":

            status_color = (
                0,
                165,
                255
            )

        else:

            status_color = (
                0,
                255,
                0
            )


        # ----------------------------------------------------
        # STATUS TEXT
        # ----------------------------------------------------

        cv2.putText(
            color,
            f"NAV: {status}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color,
            2
        )

        cv2.putText(
            color,
            (
                f"Throttle: "
                f"{decision['throttle']:.2f}  "
                f"Steer: "
                f"{decision['steer']:+.2f}"
            ),
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            status_color,
            2
        )


        # ----------------------------------------------------
        # JPEG ENCODING
        # ----------------------------------------------------

        success, buffer = (
            cv2.imencode(
                ".jpg",
                color,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    70
                ]
            )
        )

        if not success:

            continue


        frame = buffer.tobytes()


        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame
            + b"\r\n"
        )


# ============================================================
# WEB PAGE
# ============================================================

@app.route("/")
def index():

    return """
    <html>

        <head>

            <title>
                Delivery Rover Camera
            </title>

        </head>

        <body>

            <h1>
                Delivery Rover Camera
            </h1>

            <img
                src="/video_feed"
                width="640"
                height="480"
            >

        </body>

    </html>
    """


# ============================================================
# VIDEO STREAM
# ============================================================

@app.route("/video_feed")
def video_feed():

    return Response(
        generate_frames(),
        mimetype=(
            "multipart/x-mixed-replace; "
            "boundary=frame"
        )
    )


# ============================================================
# START SERVER
# ============================================================

if __name__ == "__main__":

    app.run(
        host="0.0.0.0",
        port=5000,
        threaded=True
    )
