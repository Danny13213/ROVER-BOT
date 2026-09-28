#ifndef ROVER_H
#define ROVER_H

#define ROVER_PWM_MAX 255
#define ROVER_NAV_FILE "/tmp/rover_nav.txt"

enum RoverMode {
    ROVER_MANUAL,
    ROVER_AUTO
};

enum AutoState {
    AUTO_DRIVE,
    AUTO_STOPPED,
    AUTO_SCAN,
    AUTO_TURN_TO_BEST,
    AUTO_REVERSE
};

struct NavData {
    double timestamp;
    double left;
    double center;
    double right;
    double throttle;
    double steer;
    char status[32];
};

#endif