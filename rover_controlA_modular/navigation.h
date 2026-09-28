#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "rover.h"

int navigation_parse_line(const char *line, struct NavData *nav);
int navigation_read_file(const char *path, struct NavData *nav);
int navigation_is_fresh(const struct NavData *nav, double current_time);

#endif