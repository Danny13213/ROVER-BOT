#ifndef SERIAL_H
#define SERIAL_H

int serial_open(const char *device);
void serial_send_command(int fd, char command);
void serial_set_motor_speed(int fd, char motor, int speed);

#endif