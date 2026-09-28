#include "rover_logic.h"

#include <math.h>
#include <string.h>

int rover_clamp_int(int value, int low, int high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

double rover_clamp_double(double value, double low, double high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

char rover_manual_command(char key)
{
    switch (key) {
    case 'w': return 'F';
    case 's': return 'B';
    case 'a': return 'R';
    case 'd': return 'L';
    default: return 0;
    }
}

int rover_navigation_blocked(const struct NavData *nav)
{
    return strcmp(nav->status, "STOP") == 0 || nav->throttle <= 0.0;
}

void rover_calculate_pwm(double throttle, double steer,
                         int max_left, int max_right,
                         int *left_pwm, int *right_pwm)
{
    double left_factor;
    double right_factor;

    throttle = rover_clamp_double(fabs(throttle), 0.0, 1.0);
    steer = rover_clamp_double(steer, -1.0, 1.0);
    left_factor = throttle;
    right_factor = throttle;

    if (steer < 0.0)
        left_factor *= 1.0 - fabs(steer);
    else if (steer > 0.0)
        right_factor *= 1.0 - fabs(steer);

    *left_pwm = rover_clamp_int((int)(max_left * left_factor), 0, ROVER_PWM_MAX);
    *right_pwm = rover_clamp_int((int)(max_right * right_factor), 0, ROVER_PWM_MAX);
}

int rover_scan_section(long elapsed_ms, int scan_duration_ms, int section_count)
{
    int section;

    if (scan_duration_ms <= 0 || section_count <= 0)
        return 0;

    section = (int)((double)elapsed_ms / scan_duration_ms * section_count);
    return rover_clamp_int(section, 0, section_count - 1);
}

void rover_record_clearance(double clearance, int section,
                            double *best_clearance, int *best_section)
{
    if (isfinite(clearance) && clearance > *best_clearance) {
        *best_clearance = clearance;
        *best_section = section;
    }
}