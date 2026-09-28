#define _POSIX_C_SOURCE 200809L
#ifdef __APPLE__
#define _DARWIN_C_SOURCE
#endif

#include "serial.h"

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static int configure_serial(int fd)
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

int serial_open(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY);

    if (fd < 0)
        return -1;
    if (!configure_serial(fd)) {
        close(fd);
        return -1;
    }
    return fd;
}

void serial_send_command(int fd, char command)
{
    (void)write(fd, &command, 1);
}

void serial_set_motor_speed(int fd, char motor, int speed)
{
    char command[16];
    int length;

    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    length = snprintf(command, sizeof(command), "%c%d\n", motor, speed);
    if (length > 0 && (size_t)length < sizeof(command))
        (void)write(fd, command, (size_t)length);
}