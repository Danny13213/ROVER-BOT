#define _POSIX_C_SOURCE 200809L

#include "navigation.h"
#include "rover.h"
#include "rover_logic.h"
#include "serial.h"
#include "terminal.h"
#include "time_util.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

struct Controller {
    enum RoverMode mode;
    enum AutoState auto_state;
    int left_speed;
    int right_speed;
    int moving;
    char current_movement;
    long long last_movement_command;
    long long last_auto_update;
    long long stop_started;
    long long scan_started;
    long long turn_started;
    long long reverse_started;
    long turn_duration_ms;
    double best_clearance;
    int best_section;
    int last_scan_section;
};

static void show_controls(void)
{
    printf("\n=============================\n");
    printf("        ROVER CONTROL\n");
    printf("=============================\n\n");
    printf("W = Forward\nS = Reverse\nA = Left\nD = Right\n\n");
    printf("U/J = Left PWM +/-5\nI/K = Right PWM +/-5\nP = Show PWM\n\n");
    printf("M = Manual / Auto\nSPACE = STOP\nQ = Quit\n\n");
    fflush(stdout);
}

static void set_speeds(int fd, int left, int right)
{
    serial_set_motor_speed(fd, 'A', left);
    serial_set_motor_speed(fd, 'C', right);
}

static void stop_rover(int fd, struct Controller *controller)
{
    serial_send_command(fd, 'S');
    controller->moving = 0;
    controller->current_movement = 'S';
}

static void handle_key(int fd, struct Controller *controller,
                       char key, int *running)
{
    const int speed_step = 5;

    if (key == ' ') {
        stop_rover(fd, controller);
        controller->auto_state = AUTO_DRIVE;
        printf("\n*** STOP ***\n");
    } else if (key == 'q') {
        stop_rover(fd, controller);
        *running = 0;
    } else if (key == 'm') {
        stop_rover(fd, controller);
        controller->auto_state = AUTO_DRIVE;
        if (controller->mode == ROVER_MANUAL) {
            controller->mode = ROVER_AUTO;
            printf("\nAUTO MODE\n");
        } else {
            controller->mode = ROVER_MANUAL;
            set_speeds(fd, controller->left_speed, controller->right_speed);
            printf("\nMANUAL MODE\n");
        }
    } else if (key == 'u' || key == 'j') {
        int adjustment = key == 'u' ? speed_step : -speed_step;
        controller->left_speed = rover_clamp_int(
            controller->left_speed + adjustment, 0, ROVER_PWM_MAX);
        if (controller->mode == ROVER_MANUAL)
            serial_set_motor_speed(fd, 'A', controller->left_speed);
        printf("LEFT PWM: %d\n", controller->left_speed);
    } else if (key == 'i' || key == 'k') {
        int adjustment = key == 'i' ? speed_step : -speed_step;
        controller->right_speed = rover_clamp_int(
            controller->right_speed + adjustment, 0, ROVER_PWM_MAX);
        if (controller->mode == ROVER_MANUAL)
            serial_set_motor_speed(fd, 'C', controller->right_speed);
        printf("RIGHT PWM: %d\n", controller->right_speed);
    } else if (key == 'p') {
        printf("LEFT PWM: %d RIGHT PWM: %d\n",
               controller->left_speed, controller->right_speed);
    } else if (controller->mode == ROVER_MANUAL) {
        char command = rover_manual_command(key);
        if (command != 0) {
            if (!controller->moving ||
                command != controller->current_movement) {
                serial_send_command(fd, command);
                controller->current_movement = command;
                controller->moving = 1;
            }
            controller->last_movement_command = monotonic_milliseconds();
        }
    }
    fflush(stdout);
}

static void update_manual_deadman(int fd, struct Controller *controller)
{
    const int stop_timeout_ms = 220;

    if (controller->mode == ROVER_MANUAL && controller->moving &&
        monotonic_milliseconds() - controller->last_movement_command >
            stop_timeout_ms) {
        stop_rover(fd, controller);
    }
}

static void update_auto_drive(int fd, struct Controller *controller,
                              long long now)
{
    const int update_interval_ms = 100;
    struct NavData nav;
    int left_pwm;
    int right_pwm;

    if (now - controller->last_auto_update < update_interval_ms)
        return;
    controller->last_auto_update = now;

    if (!navigation_read_file(ROVER_NAV_FILE, &nav) ||
        !navigation_is_fresh(&nav, realtime_seconds())) {
        serial_send_command(fd, 'S');
        printf("AUTO: CAMERA ERROR -> STOP\n");
        return;
    }

    if (rover_navigation_blocked(&nav)) {
        serial_send_command(fd, 'S');
        controller->auto_state = AUTO_STOPPED;
        controller->stop_started = now;
        printf("AUTO: BLOCKED\n");
        return;
    }

    rover_calculate_pwm(nav.throttle, nav.steer,
                        controller->left_speed, controller->right_speed,
                        &left_pwm, &right_pwm);
    set_speeds(fd, left_pwm, right_pwm);
    serial_send_command(fd, 'F');
    printf("AUTO: %s C=%.2fm steer=%.3f PWM=%d/%d\n",
           nav.status, nav.center, nav.steer, left_pwm, right_pwm);
}

static void begin_scan(int fd, struct Controller *controller, long long now)
{
    const int scan_pwm = 110;

    printf("\nSTARTING 360 SCAN\n");
    controller->best_clearance = 0.0;
    controller->best_section = 0;
    controller->last_scan_section = -1;
    controller->scan_started = now;
    set_speeds(fd, scan_pwm, scan_pwm);
    serial_send_command(fd, 'R');
    controller->auto_state = AUTO_SCAN;
}

static void update_stopped(int fd, struct Controller *controller,
                           long long now)
{
    const int stuck_time_ms = 800;

    serial_send_command(fd, 'S');
    if (now - controller->stop_started >= stuck_time_ms)
        begin_scan(fd, controller, now);
}

static void sample_scan(struct Controller *controller, int section)
{
    struct NavData nav;

    controller->last_scan_section = section;
    if (navigation_read_file(ROVER_NAV_FILE, &nav) &&
        navigation_is_fresh(&nav, realtime_seconds())) {
        rover_record_clearance(nav.center, section,
                               &controller->best_clearance,
                               &controller->best_section);
        printf("SCAN %d center=%.2fm\n", section, nav.center);
    }
}

static void finish_scan(int fd, struct Controller *controller, long long now)
{
    const double minimum_clearance = 1.50;
    const int reverse_pwm = 70;
    const int scan_pwm = 110;
    const int scan_duration_ms = 4000;
    const int section_count = 16;

    serial_send_command(fd, 'S');
    printf("\nSCAN COMPLETE\nBest clearance: %.2fm\n",
           controller->best_clearance);

    if (controller->best_clearance < minimum_clearance) {
        printf("No safe route. Short reverse.\n");
        set_speeds(fd, reverse_pwm, reverse_pwm);
        serial_send_command(fd, 'B');
        controller->reverse_started = now;
        controller->auto_state = AUTO_REVERSE;
    } else {
        double fraction = (double)controller->best_section / section_count;
        controller->turn_duration_ms = (long)(scan_duration_ms * fraction);
        printf("Best scan section: %d\nTurning toward opening...\n",
               controller->best_section);
        set_speeds(fd, scan_pwm, scan_pwm);
        serial_send_command(fd, 'R');
        controller->turn_started = now;
        controller->auto_state = AUTO_TURN_TO_BEST;
    }
}

static void update_scan(int fd, struct Controller *controller, long long now)
{
    const int scan_duration_ms = 4000;
    const int section_count = 16;
    long long elapsed = now - controller->scan_started;
    int section = rover_scan_section(elapsed, scan_duration_ms, section_count);

    if (section != controller->last_scan_section)
        sample_scan(controller, section);
    if (elapsed >= scan_duration_ms)
        finish_scan(fd, controller, now);
}

static void finish_timed_state(int fd, struct Controller *controller,
                               long long now)
{
    const int reverse_time_ms = 700;

    if (controller->auto_state == AUTO_TURN_TO_BEST &&
        now - controller->turn_started >= controller->turn_duration_ms) {
        serial_send_command(fd, 'S');
        printf("OPENING SELECTED\n");
        set_speeds(fd, controller->left_speed, controller->right_speed);
        controller->auto_state = AUTO_DRIVE;
        controller->last_auto_update = now;
    } else if (controller->auto_state == AUTO_REVERSE &&
               now - controller->reverse_started >= reverse_time_ms) {
        serial_send_command(fd, 'S');
        printf("REVERSE COMPLETE\n");
        set_speeds(fd, controller->left_speed, controller->right_speed);
        controller->auto_state = AUTO_DRIVE;
        controller->last_auto_update = now;
    }
}

static void update_autonomous(int fd, struct Controller *controller)
{
    long long now = monotonic_milliseconds();

    switch (controller->auto_state) {
    case AUTO_DRIVE:
        update_auto_drive(fd, controller, now);
        break;
    case AUTO_STOPPED:
        update_stopped(fd, controller, now);
        break;
    case AUTO_SCAN:
        update_scan(fd, controller, now);
        break;
    case AUTO_TURN_TO_BEST:
    case AUTO_REVERSE:
        finish_timed_state(fd, controller, now);
        break;
    }
}

int main(int argc, char **argv)
{
    const char *device = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    struct Controller controller = {
        .mode = ROVER_MANUAL,
        .auto_state = AUTO_DRIVE,
        .left_speed = 200,
        .right_speed = 200,
        .current_movement = 'S',
        .last_scan_section = -1
    };
    struct Terminal terminal = {0};
    int serial_fd = serial_open(device);
    int running = 1;

    if (serial_fd < 0) {
        fprintf(stderr, "ERROR: Could not open/configure serial device %s\n",
                device);
        return 1;
    }

    terminal_start(&terminal);
    controller.last_movement_command = monotonic_milliseconds();
    controller.last_auto_update = controller.last_movement_command;
    serial_send_command(serial_fd, 'S');
    set_speeds(serial_fd, controller.left_speed, controller.right_speed);
    sleep_milliseconds(50);
    show_controls();

    while (running) {
        char key;
        if (terminal_read_key(&key))
            handle_key(serial_fd, &controller, key, &running);

        update_manual_deadman(serial_fd, &controller);
        if (controller.mode == ROVER_AUTO)
            update_autonomous(serial_fd, &controller);
        sleep_milliseconds(5);
    }

    serial_send_command(serial_fd, 'S');
    terminal_restore(&terminal);
    close(serial_fd);
    printf("\nController closed.\n");
    return 0;
}