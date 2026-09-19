"""Depth-zone navigation logic for the delivery rover.

Zones are ordered:
    LEFT, CENTER, RIGHT

Distances are in metres.

float("inf") means the zone has insufficient valid depth data.
For the Astra Pro this is treated as blocked/unsafe.
"""

import numpy as np


# ============================================================
# NAVIGATION THRESHOLDS
# ============================================================

DIST_STOP_M = 0.50
DIST_CAUTION_M = 1.50

AVOID_MIN_CLEARANCE_M = 0.70


# ============================================================
# DEPTH CAMERA ANALYSIS SETTINGS
# ============================================================

ZONE_MIN_VALID_PX = 200

BAND_TOP_FRAC = 0.30
BAND_BOT_FRAC = 0.85


# ============================================================
# BLOCKED CHECK
# ============================================================

def is_blocked(dist_m):

    return (
        dist_m <= DIST_STOP_M
        or dist_m == float("inf")
    )


# ============================================================
# DEPTH ZONE ANALYSIS
# ============================================================

def analyze_depth_zones(depth_mm):

    H, W = depth_mm.shape

    y0 = int(H * BAND_TOP_FRAC)
    y1 = int(H * BAND_BOT_FRAC)

    band = depth_mm[y0:y1, :]

    zone_width = W // 3

    zones = []

    for i in range(3):

        x0 = i * zone_width

        if i < 2:
            x1 = (i + 1) * zone_width
        else:
            x1 = W

        zone = band[:, x0:x1]

        valid = zone[zone > 0]

        if valid.size < ZONE_MIN_VALID_PX:

            zones.append(
                float("inf")
            )

        else:

            distance_m = (
                float(
                    np.percentile(
                        valid,
                        5
                    )
                )
                / 1000.0
            )

            zones.append(
                distance_m
            )

    return tuple(zones)


# ============================================================
# NAVIGATION DECISION
# ============================================================

def decide(zones):

    left_m, center_m, right_m = zones

    blocked = [
        is_blocked(left_m),
        is_blocked(center_m),
        is_blocked(right_m)
    ]

    n_blocked = sum(blocked)


    # --------------------------------------------------------
    # EVERYTHING BLOCKED
    # --------------------------------------------------------

    if n_blocked == 3:

        return {
            "status": "STOP",
            "throttle": 0.0,
            "steer": 0.0,
            "action": "STOP"
        }


    # --------------------------------------------------------
    # CENTER BLOCKED
    # --------------------------------------------------------

    if blocked[1]:

        # Both sides available
        if (
            not blocked[0]
            and not blocked[2]
        ):

            if left_m > right_m:
                steer = -0.6
            else:
                steer = 0.6


        # Only left available
        elif not blocked[0]:

            if (
                left_m
                >= AVOID_MIN_CLEARANCE_M
            ):

                steer = -0.6

            else:

                return {
                    "status": "STOP",
                    "throttle": 0.0,
                    "steer": 0.0,
                    "action": "STOP"
                }


        # Only right available
        elif not blocked[2]:

            if (
                right_m
                >= AVOID_MIN_CLEARANCE_M
            ):

                steer = 0.6

            else:

                return {
                    "status": "STOP",
                    "throttle": 0.0,
                    "steer": 0.0,
                    "action": "STOP"
                }


        else:

            return {
                "status": "STOP",
                "throttle": 0.0,
                "steer": 0.0,
                "action": "STOP"
            }


        return {
            "status": "AVOID",
            "throttle": 0.3,
            "steer": steer,
            "action": "AVOID"
        }


    # --------------------------------------------------------
    # CENTER OPEN BUT CLOSE TO OBSTACLE
    # --------------------------------------------------------

    if center_m <= DIST_CAUTION_M:

        if (
            not blocked[0]
            and not blocked[2]
        ):

            if left_m > right_m:
                steer = -0.6
            else:
                steer = 0.6


        elif not blocked[0]:

            steer = -0.6


        elif not blocked[2]:

            steer = 0.6


        else:

            steer = 0.0


        return {
            "status": "AVOID",
            "throttle": 0.3,
            "steer": steer,
            "action": "AVOID"
        }


    # --------------------------------------------------------
    # CENTER CLEAR
    # --------------------------------------------------------

    if (
        blocked[0]
        and not blocked[2]
    ):

        # Left unsafe → bias right
        steer = 0.2


    elif (
        blocked[2]
        and not blocked[0]
    ):

        # Right unsafe → bias left
        steer = -0.2


    elif (
        not blocked[0]
        and not blocked[2]
    ):

        steer = float(
            np.clip(
                (
                    right_m
                    - left_m
                )
                /
                max(
                    left_m,
                    right_m,
                    1.0
                ),
                -0.3,
                0.3
            )
        )


    else:

        steer = 0.0


    return {
        "status": "GO",
        "throttle": 0.7,
        "steer": steer,
        "action": "GO"
    }
