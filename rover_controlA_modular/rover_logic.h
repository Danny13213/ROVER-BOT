#ifndef ROVER_LOGIC_H
#define ROVER_LOGIC_H

#include "rover.h"

int rover_clamp_int(int value, int low, int high);
double rover_clamp_double(double value, double low, double high);
char rover_manual_command(char key);
int rover_navigation_blocked(const struct NavData *nav);
void rover_calculate_pwm(double throttle, double steer,
                         int max_left, int max_right,
                         int *left_pwm, int *right_pwm);
int rover_scan_section(long elapsed_ms, int scan_duration_ms, int section_count);
void rover_record_clearance(double clearance, int section,
                            double *best_clearance, int *best_section);

#endif