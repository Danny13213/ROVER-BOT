#include "navigation.h"
#include "rover.h"
#include "rover_logic.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_manual_commands(void)
{
    assert(rover_manual_command('w') == 'F');
    assert(rover_manual_command('s') == 'B');
    assert(rover_manual_command('a') == 'R');
    assert(rover_manual_command('d') == 'L');
    assert(rover_manual_command('x') == 0);
}

static void test_pwm_calculation(void)
{
    int left;
    int right;

    rover_calculate_pwm(0.5, 0.0, 200, 160, &left, &right);
    assert(left == 100 && right == 80);

    rover_calculate_pwm(1.0, 1.0, 200, 200, &left, &right);
    assert(left == 200 && right == 0);

    rover_calculate_pwm(2.0, -2.0, 300, 300, &left, &right);
    assert(left == 0 && right == ROVER_PWM_MAX);
}

static void test_scan_and_clearance(void)
{
    double best = 0.0;
    int best_section = 0;

    assert(rover_scan_section(0, 4000, 16) == 0);
    assert(rover_scan_section(1000, 4000, 16) == 4);
    assert(rover_scan_section(4000, 4000, 16) == 15);

    rover_record_clearance(2.0, 3, &best, &best_section);
    rover_record_clearance(1.5, 6, &best, &best_section);
    rover_record_clearance(INFINITY, 9, &best, &best_section);
    assert(best == 2.0 && best_section == 3);
}

static void test_navigation_data(void)
{
    struct NavData nav;
    const char *sample = "1000.0 2.1 1.8 2.4 0.7 -0.2 CLEAR";

    assert(navigation_parse_line(sample, &nav));
    assert(nav.timestamp == 1000.0);
    assert(nav.center == 1.8);
    assert(navigation_is_fresh(&nav, 1001.0));
    assert(!navigation_is_fresh(&nav, 1003.1));
    assert(!navigation_is_fresh(&nav, 999.0));
    assert(!navigation_parse_line("not navigation data", &nav));
}

int main(void)
{
    test_manual_commands();
    test_pwm_calculation();
    test_scan_and_clearance();
    test_navigation_data();
    puts("All rover logic and navigation tests passed.");
    return 0;
}