import sys
import os
import time

import cv2
import numpy as np

from flask import Flask, Response
from ultralytics import YOLO


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

print("Starting Astra camera...")
cam = AstraCamera()
print("Astra camera ready.")


# ============================================================
# YOLO
# ============================================================

print("Loading YOLO...")

yolo_model = YOLO("yolo11n.pt")

print("YOLO loaded.")

YOLO_CONFIDENCE = 0.50

# Run YOLO every N frames.
# Depth navigation still runs every frame.
YOLO_EVERY_N_FRAMES = 3

frame_counter = 0

# Keep the most recent detections between YOLO frames.
last_detections = []


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
# YOLO DETECTION
# ============================================================

def run_yolo(frame):

    detections = []

    try:

        results = yolo_model.predict(
            source=frame,
            conf=YOLO_CONFIDENCE,
            device=0,
            verbose=False
        )

        result = results[0]

        for box in result.boxes:

            class_id = int(
                box.cls[0].item()
            )

            confidence = float(
                box.conf[0].item()
            )

            x1, y1, x2, y2 = (
                box.xyxy[0]
                .cpu()
                .numpy()
                .astype(int)
            )

            name = yolo_model.names[
                class_id
            ]

            detections.append(
                {
                    "name": name,
                    "confidence": confidence,
                    "box": (
                        x1,
                        y1,
                        x2,
                        y2
                    )
                }
            )

    except Exception as e:

        print(
            "YOLO error:",
            e
        )

    return detections


# ============================================================
# OBJECT DEPTH
# ============================================================

def object_distance(
    depth_mm,
    box
):

    x1, y1, x2, y2 = box

    h, w = depth_mm.shape[:2]

    # Keep coordinates inside image.
    x1 = max(
        0,
        min(x1, w - 1)
    )

    x2 = max(
        0,
        min(x2, w)
    )

    y1 = max(
        0,
        min(y1, h - 1)
    )

    y2 = max(
        0,
        min(y2, h)
    )

    if x2 <= x1 or y2 <= y1:
        return None

    # Use the center portion of the YOLO box instead of
    # the entire box. This reduces background depth pixels.

    box_width = x2 - x1
    box_height = y2 - y1

    cx1 = int(
        x1 + box_width * 0.25
    )

    cx2 = int(
        x2 - box_width * 0.25
    )

    cy1 = int(
        y1 + box_height * 0.25
    )

    cy2 = int(
        y2 - box_height * 0.25
    )

    region = depth_mm[
        cy1:cy2,
        cx1:cx2
    ]

    if region.size == 0:
        return None

    # Ignore invalid depth pixels.
    valid = region[
        region > 0
    ]

    if valid.size < 20:
        return None

    # Median is more resistant to bad depth pixels.
    distance_mm = np.median(
        valid
    )

    return float(
        distance_mm
    ) / 1000.0


# ============================================================
# DRAW YOLO DETECTIONS
# ============================================================

def draw_detections(
    frame,
    depth_mm,
    detections
):

    for detection in detections:

        name = detection["name"]

        confidence = detection[
            "confidence"
        ]

        box = detection["box"]

        x1, y1, x2, y2 = box

        distance = object_distance(
            depth_mm,
            box
        )

        # Bounding box
        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 255),
            2
        )

        if distance is not None:

            label = (
                f"{name} "
                f"{confidence:.2f} "
                f"{distance:.2f}m"
            )

        else:

            label = (
                f"{name} "
                f"{confidence:.2f}"
            )

        # Keep text on screen.
        text_y = max(
            20,
            y1 - 10
        )

        cv2.putText(
            frame,
            label,
            (
                x1,
                text_y
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2
        )


# ============================================================
# FRAME GENERATOR
# ============================================================

def generate_frames():

    global frame_counter
    global last_detections

    while True:

        # ----------------------------------------------------
        # READ ASTRA
        # ----------------------------------------------------

        try:

            color, depth_mm = (
                cam.read()
            )

        except Exception as e:

            print(
                "Camera error:",
                e
            )

            time.sleep(0.05)

            continue


        # ----------------------------------------------------
        # DEPTH NAVIGATION
        #
        # This still runs EVERY frame.
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

        status = decision[
            "status"
        ]


        # ----------------------------------------------------
        # PUBLISH MOTOR NAVIGATION
        #
        # Do this BEFORE YOLO so AI processing does not delay
        # the navigation update unnecessarily.
        # ----------------------------------------------------

        publish_navigation(
            left_m,
            center_m,
            right_m,
            decision
        )


        # ----------------------------------------------------
        # YOLO
        # ----------------------------------------------------

        frame_counter += 1

        if (
            frame_counter
            % YOLO_EVERY_N_FRAMES
            == 0
        ):

            last_detections = (
                run_yolo(
                    color
                )
            )


        # ----------------------------------------------------
        # DRAW YOLO OBJECTS
        # ----------------------------------------------------

        draw_detections(
            color,
            depth_mm,
            last_detections
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
            (
                zone_width,
                y0
            ),
            (
                zone_width,
                y1
            ),
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
        # LEFT
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
        # CENTER
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
        # RIGHT
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
        # STATUS
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
        # AI STATUS
        # ----------------------------------------------------

        cv2.putText(
            color,
            (
                f"AI objects: "
                f"{len(last_detections)}"
            ),
            (
                20,
                h - 20
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2
        )


        # ----------------------------------------------------
        # JPEG
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
                AI Delivery Rover
            </title>

        </head>

        <body>

            <h1>
                AI Delivery Rover Camera
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

    try:

        app.run(
            host="0.0.0.0",
            port=5000,
            threaded=True
        )

    finally:

        try:
            cam.release()
        except Exception:
            pass
