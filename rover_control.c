/* Original C++ reference: rover_control.cpp is kept unchanged beside this port. */

#define _POSIX_C_SOURCE 200809L
#ifdef __APPLE__
#define _DARWIN_C_SOURCE
#endif

#include <fcntl.h>
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

static long long monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)now.tv_sec * 1000 + now.tv_nsec / 1000000;
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

int main(void)
{
    const char *serial_device = "/dev/ttyUSB0";
    const int speed_step = 5;
    const long stop_timeout_ms = 220;
    int serial_fd;
    int old_flags;
    int terminal_saved = 0;
    int flags_saved = 0;
    int running = 1;
    int moving = 0;
    int left_speed = 200;
    int right_speed = 200;
    char current_movement = 'S';
    enum Mode mode = MANUAL;
    long long last_movement_command;
    struct termios old_terminal;
    struct termios new_terminal;

    serial_fd = open(serial_device, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        fprintf(stderr, "ERROR: Could not open %s\n", serial_device);
        return 1;
    }

    if (!setup_serial(serial_fd)) {
        fprintf(stderr, "ERROR: Could not configure serial port.\n");
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

    send_command(serial_fd, 'S');
    set_motor_speed(serial_fd, 'A', left_speed);
    sleep_ms(60);
    set_motor_speed(serial_fd, 'C', right_speed);

    printf("\n=============================\n");
    printf("        ROVER CONTROL\n");
    printf("=============================\n\n");
    printf("Hold W = Forward\nHold A = Left\nHold D = Right\n\n");
    printf("U = Left PWM +5\nJ = Left PWM -5\n");
    printf("I = Right PWM +5\nK = Right PWM -5\nP = Show PWM\n\n");
    printf("SPACE = STOP\nM = Manual / Auto\nQ = Quit\n\n");
    printf("MODE: MANUAL\nLEFT PWM:  %d\nRIGHT PWM: %d\n", left_speed, right_speed);
    printf("=============================\n\n");
    fflush(stdout);

    last_movement_command = monotonic_ms();

    while (running) {
        char key;
        ssize_t bytes = read(STDIN_FILENO, &key, 1);

        if (bytes > 0) {
            if (key >= 'A' && key <= 'Z')
                key = (char)(key - 'A' + 'a');

            if (key == ' ') {
                send_command(serial_fd, 'S');
                moving = 0;
                current_movement = 'S';
                printf("\n*** STOP ***\n");
            } else if (key == 'q') {
                send_command(serial_fd, 'S');
                printf("\nStopping rover...\n");
                running = 0;
            } else if (key == 'm') {
                send_command(serial_fd, 'S');
                moving = 0;
                current_movement = 'S';
                if (mode == MANUAL) {
                    mode = AUTO;
                    printf("\n=============================\n");
                    printf("       AUTONOMOUS MODE\n");
                    printf("=============================\n");
                    printf("Rover stopped.\nAutonomous navigation not enabled yet.\n\n");
                } else {
                    mode = MANUAL;
                    printf("\n=============================\n");
                    printf("          MANUAL MODE\n");
                    printf("=============================\n\n");
                }
            } else if (key == 'u') {
                left_speed += speed_step;
                if (left_speed > 255)
                    left_speed = 255;
                set_motor_speed(serial_fd, 'A', left_speed);
                printf("LEFT PWM: %d\n", left_speed);
            } else if (key == 'j') {
                left_speed -= speed_step;
                if (left_speed < 0)
                    left_speed = 0;
                set_motor_speed(serial_fd, 'A', left_speed);
                printf("LEFT PWM: %d\n", left_speed);
            } else if (key == 'i') {
                right_speed += speed_step;
                if (right_speed > 255)
                    right_speed = 255;
                set_motor_speed(serial_fd, 'C', right_speed);
                printf("RIGHT PWM: %d\n", right_speed);
            } else if (key == 'k') {
                right_speed -= speed_step;
                if (right_speed < 0)
                    right_speed = 0;
                set_motor_speed(serial_fd, 'C', right_speed);
                printf("RIGHT PWM: %d\n", right_speed);
            } else if (key == 'p') {
                printf("\nLEFT PWM:  %d\nRIGHT PWM: %d\n\n", left_speed, right_speed);
            } else if (mode == MANUAL) {
                char command = 0;
                if (key == 'w')
                    command = 'F';
                else if (key == 'a')
                    command = 'R';
                else if (key == 'd')
                    command = 'L';

                if (command != 0) {
                    if (!moving || command != current_movement) {
                        send_command(serial_fd, command);
                        current_movement = command;
                        moving = 1;
                        if (key == 'w')
                            printf("FORWARD  L=%d R=%d\n", left_speed, right_speed);
                        else if (key == 'a')
                            printf("LEFT  L=%d R=%d\n", left_speed, right_speed);
                        else
                            printf("RIGHT  L=%d R=%d\n", left_speed, right_speed);
                    }
                    last_movement_command = monotonic_ms();
                }
            }
            fflush(stdout);
        }

        if (mode == MANUAL && moving &&
            monotonic_ms() - last_movement_command > stop_timeout_ms) {
            send_command(serial_fd, 'S');
            moving = 0;
            current_movement = 'S';
            printf("STOP\n");
            fflush(stdout);
        }

        sleep_ms(5);
    }

    send_command(serial_fd, 'S');
    if (terminal_saved)
        (void)tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
    if (flags_saved)
        (void)fcntl(STDIN_FILENO, F_SETFL, old_flags);
    close(serial_fd);
    printf("Controller closed.\n");
    return 0;
}