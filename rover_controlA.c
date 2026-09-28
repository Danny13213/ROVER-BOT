/* Original C++ reference: rover_controlA.cpp is kept unchanged beside this port. */

#define _POSIX_C_SOURCE 200809L
#ifdef __APPLE__
#define _DARWIN_C_SOURCE
#endif

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

enum Mode {
    MANUAL,
    AUTO
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

static long long monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static double realtime_seconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}

static void sleep_ms(long milliseconds)
{
    struct timespec delay = {
        .tv_sec = milliseconds / 1000,
        .tv_nsec = (milliseconds % 1000) * 1000000L
    };
    nanosleep(&delay, NULL);
}

static void send_command(int serial_fd, char command)
{
    (void)write(serial_fd, &command, 1);
}

static void set_motor_speed(int serial_fd, char motor, int speed)
{
    char command[16];
    int length;

    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    length = snprintf(command, sizeof(command), "%c%d\n", motor, speed);
    if (length > 0 && (size_t)length < sizeof(command))
        (void)write(serial_fd, command, (size_t)length);
}

static int setup_serial(int fd)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0)
        return 0;

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag &= (tcflag_t)~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_iflag &= (tcflag_t)~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= (tcflag_t)~PARENB;
    tty.c_cflag &= (tcflag_t)~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= (tcflag_t)~CRTSCTS;
#endif
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

static int read_navigation(struct NavData *nav)
{
    FILE *file = fopen("/tmp/rover_nav.txt", "r");
    int fields_read;

    if (file == NULL)
        return 0;

    fields_read = fscanf(file, "%lf %lf %lf %lf %lf %lf %31s",
                         &nav->timestamp, &nav->left, &nav->center,
                         &nav->right, &nav->throttle, &nav->steer,
                         nav->status);
    fclose(file);
    return fields_read == 7;
}

static int navigation_fresh(const struct NavData *nav)
{
    double age = realtime_seconds() - nav->timestamp;
    return age >= 0.0 && age <= 2.0;
}

static int clamp_int(int value, int low, int high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static double clamp_double(double value, double low, double high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static void calculate_auto_pwm(double throttle, double steer,
                               int max_left, int max_right,
                               int *left_pwm, int *right_pwm)
{
    double left_factor;
    double right_factor;

    throttle = clamp_double(fabs(throttle), 0.0, 1.0);
    steer = clamp_double(steer, -1.0, 1.0);
    left_factor = throttle;
    right_factor = throttle;

    if (steer < 0.0)
        left_factor *= 1.0 - fabs(steer);
    else if (steer > 0.0)
        right_factor *= 1.0 - fabs(steer);

    *left_pwm = clamp_int((int)(max_left * left_factor), 0, 255);
    *right_pwm = clamp_int((int)(max_right * right_factor), 0, 255);
}

int main(void)
{
    const char *serial_device = "/dev/ttyUSB0";
    const int speed_step = 5;
    const int scan_pwm = 110;
    const int full_scan_ms = 4000;
    const int scan_sections = 16;
    const double min_good_clearance = 1.50;
    const int reverse_pwm = 70;
    const int reverse_time_ms = 700;
    const int stuck_time_ms = 800;
    const int stop_timeout_ms = 220;
    const int auto_update_interval_ms = 100;
    int serial_fd;
    int old_flags;
    int terminal_saved = 0;
    int flags_saved = 0;
    int running = 1;
    int moving = 0;
    int left_speed = 200;
    int right_speed = 200;
    int auto_left;
    int auto_right;
    int best_section = 0;
    int last_scan_section = -1;
    long long now;
    long long last_movement_command;
    long long last_auto_update;
    long long stop_start = 0;
    long long scan_start = 0;
    long long turn_start = 0;
    long long reverse_start = 0;
    long turn_duration_ms = 0;
    double best_clearance = 0.0;
    char current_movement = 'S';
    enum Mode mode = MANUAL;
    enum AutoState auto_state = AUTO_DRIVE;
    struct termios old_terminal;
    struct termios new_terminal;

    serial_fd = open(serial_device, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        fprintf(stderr, "ERROR: Could not open %s\n", serial_device);
        return 1;
    }

    if (!setup_serial(serial_fd)) {
        fprintf(stderr, "ERROR: Serial setup failed.\n");
        close(serial_fd);
        return 1;
    }

    if (tcgetattr(STDIN_FILENO, &old_terminal) == 0) {
        terminal_saved = 1;
        new_terminal = old_terminal;
        new_terminal.c_lflag &= (tcflag_t)~(ICANON | ECHO);
        new_terminal.c_cc[VMIN] = 0;
        new_terminal.c_cc[VTIME] = 0;
        (void)tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
    }

    old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags >= 0) {
        flags_saved = 1;
        (void)fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
    }

    now = monotonic_ms();
    last_movement_command = now;
    last_auto_update = now;

    send_command(serial_fd, 'S');
    set_motor_speed(serial_fd, 'A', left_speed);
    sleep_ms(50);
    set_motor_speed(serial_fd, 'C', right_speed);

    printf("\n=============================\n");
    printf("        ROVER CONTROL\n");
    printf("=============================\n\n");
    printf("W = Forward\nS = Reverse\nA = Left\nD = Right\n\n");
    printf("U = Left PWM +5\nJ = Left PWM -5\n");
    printf("I = Right PWM +5\nK = Right PWM -5\nP = Show PWM\n\n");
    printf("M = Manual / Auto\nSPACE = STOP\nQ = Quit\n\n");
    fflush(stdout);

    while (running) {
        char key;
        ssize_t bytes = read(STDIN_FILENO, &key, 1);

        if (bytes > 0) {
            if (key >= 'A' && key <= 'Z')
                key = (char)(key - 'A' + 'a');

            if (key == ' ') {
                send_command(serial_fd, 'S');
                moving = 0;
                auto_state = AUTO_DRIVE;
                printf("\n*** STOP ***\n");
            } else if (key == 'q') {
                send_command(serial_fd, 'S');
                running = 0;
            } else if (key == 'm') {
                send_command(serial_fd, 'S');
                moving = 0;
                auto_state = AUTO_DRIVE;
                if (mode == MANUAL) {
                    mode = AUTO;
                    printf("\nAUTO MODE\n");
                } else {
                    mode = MANUAL;
                    set_motor_speed(serial_fd, 'A', left_speed);
                    set_motor_speed(serial_fd, 'C', right_speed);
                    printf("\nMANUAL MODE\n");
                }
            } else if (key == 'u') {
                left_speed = clamp_int(left_speed + speed_step, 0, 255);
                if (mode == MANUAL)
                    set_motor_speed(serial_fd, 'A', left_speed);
                printf("LEFT PWM: %d\n", left_speed);
            } else if (key == 'j') {
                left_speed = clamp_int(left_speed - speed_step, 0, 255);
                if (mode == MANUAL)
                    set_motor_speed(serial_fd, 'A', left_speed);
                printf("LEFT PWM: %d\n", left_speed);
            } else if (key == 'i') {
                right_speed = clamp_int(right_speed + speed_step, 0, 255);
                if (mode == MANUAL)
                    set_motor_speed(serial_fd, 'C', right_speed);
                printf("RIGHT PWM: %d\n", right_speed);
            } else if (key == 'k') {
                right_speed = clamp_int(right_speed - speed_step, 0, 255);
                if (mode == MANUAL)
                    set_motor_speed(serial_fd, 'C', right_speed);
                printf("RIGHT PWM: %d\n", right_speed);
            } else if (key == 'p') {
                printf("LEFT PWM: %d RIGHT PWM: %d\n", left_speed, right_speed);
            } else if (mode == MANUAL) {
                char command = 0;
                if (key == 'w')
                    command = 'F';
                else if (key == 's')
                    command = 'B';
                else if (key == 'a')
                    command = 'R';
                else if (key == 'd')
                    command = 'L';

                if (command != 0) {
                    if (!moving || command != current_movement) {
                        send_command(serial_fd, command);
                        current_movement = command;
                        moving = 1;
                    }
                    last_movement_command = monotonic_ms();
                }
            }
            fflush(stdout);
        }

        now = monotonic_ms();
        if (mode == MANUAL && moving &&
            now - last_movement_command > stop_timeout_ms) {
            send_command(serial_fd, 'S');
            moving = 0;
            current_movement = 'S';
        }

        if (mode == AUTO) {
            now = monotonic_ms();

            if (auto_state == AUTO_DRIVE) {
                if (now - last_auto_update >= auto_update_interval_ms) {
                    struct NavData nav;
                    last_auto_update = now;

                    if (!read_navigation(&nav) || !navigation_fresh(&nav)) {
                        send_command(serial_fd, 'S');
                        printf("AUTO: CAMERA ERROR -> STOP\n");
                    } else if (strcmp(nav.status, "STOP") == 0 || nav.throttle <= 0.0) {
                        send_command(serial_fd, 'S');
                        auto_state = AUTO_STOPPED;
                        stop_start = now;
                        printf("AUTO: BLOCKED\n");
                    } else {
                        calculate_auto_pwm(nav.throttle, nav.steer,
                                           left_speed, right_speed,
                                           &auto_left, &auto_right);
                        set_motor_speed(serial_fd, 'A', auto_left);
                        set_motor_speed(serial_fd, 'C', auto_right);
                        send_command(serial_fd, 'F');
                        printf("AUTO: %s C=%.2fm steer=%.3f PWM=%d/%d\n",
                               nav.status, nav.center, nav.steer,
                               auto_left, auto_right);
                    }
                }
            } else if (auto_state == AUTO_STOPPED) {
                send_command(serial_fd, 'S');
                if (now - stop_start >= stuck_time_ms) {
                    printf("\nSTARTING 360 SCAN\n");
                    best_clearance = 0.0;
                    best_section = 0;
                    last_scan_section = -1;
                    scan_start = now;
                    set_motor_speed(serial_fd, 'A', scan_pwm);
                    set_motor_speed(serial_fd, 'C', scan_pwm);
                    send_command(serial_fd, 'R');
                    auto_state = AUTO_SCAN;
                }
            } else if (auto_state == AUTO_SCAN) {
                long long elapsed = now - scan_start;
                double progress = (double)elapsed / full_scan_ms;
                int section = (int)(progress * scan_sections);
                struct NavData nav;

                section = clamp_int(section, 0, scan_sections - 1);
                if (section != last_scan_section) {
                    last_scan_section = section;
                    if (read_navigation(&nav) && navigation_fresh(&nav)) {
                        double clearance = nav.center;
                        if (!isinf(clearance) && clearance > best_clearance) {
                            best_clearance = clearance;
                            best_section = section;
                        }
                        printf("SCAN %d/%d center=%.2fm\n",
                               section, scan_sections, clearance);
                    }
                }

                if (elapsed >= full_scan_ms) {
                    send_command(serial_fd, 'S');
                    printf("\nSCAN COMPLETE\nBest clearance: %.2fm\n", best_clearance);
                    if (best_clearance < min_good_clearance) {
                        printf("No safe route. Short reverse.\n");
                        set_motor_speed(serial_fd, 'A', reverse_pwm);
                        set_motor_speed(serial_fd, 'C', reverse_pwm);
                        send_command(serial_fd, 'B');
                        reverse_start = now;
                        auto_state = AUTO_REVERSE;
                    } else {
                        double fraction = (double)best_section / scan_sections;
                        turn_duration_ms = (long)(full_scan_ms * fraction);
                        printf("Best scan section: %d\nTurning toward opening...\n",
                               best_section);
                        set_motor_speed(serial_fd, 'A', scan_pwm);
                        set_motor_speed(serial_fd, 'C', scan_pwm);
                        send_command(serial_fd, 'R');
                        turn_start = now;
                        auto_state = AUTO_TURN_TO_BEST;
                    }
                }
            } else if (auto_state == AUTO_TURN_TO_BEST) {
                if (now - turn_start >= turn_duration_ms) {
                    send_command(serial_fd, 'S');
                    printf("OPENING SELECTED\n");
                    set_motor_speed(serial_fd, 'A', left_speed);
                    set_motor_speed(serial_fd, 'C', right_speed);
                    auto_state = AUTO_DRIVE;
                    last_auto_update = now;
                }
            } else if (auto_state == AUTO_REVERSE) {
                if (now - reverse_start >= reverse_time_ms) {
                    send_command(serial_fd, 'S');
                    printf("REVERSE COMPLETE\n");
                    set_motor_speed(serial_fd, 'A', left_speed);
                    set_motor_speed(serial_fd, 'C', right_speed);
                    auto_state = AUTO_DRIVE;
                    last_auto_update = now;
                }
            }
        }

        sleep_ms(5);
    }

    send_command(serial_fd, 'S');
    if (terminal_saved)
        (void)tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
    if (flags_saved)
        (void)fcntl(STDIN_FILENO, F_SETFL, old_flags);
    close(serial_fd);
    printf("\nController closed.\n");
    return 0;
}