#include "navigation.h"

#include <math.h>
#include <stdio.h>

int navigation_parse_line(const char *line, struct NavData *nav)
{
    return sscanf(line, "%lf %lf %lf %lf %lf %lf %31s",
                  &nav->timestamp, &nav->left, &nav->center,
                  &nav->right, &nav->throttle, &nav->steer,
                  nav->status) == 7;
}

int navigation_read_file(const char *path, struct NavData *nav)
{
    char line[256];
    FILE *file = fopen(path, "r");
    int parsed;

    if (file == NULL)
        return 0;

    parsed = fgets(line, sizeof(line), file) != NULL &&
             navigation_parse_line(line, nav);
    fclose(file);
    return parsed;
}

int navigation_is_fresh(const struct NavData *nav, double current_time)
{
    double age = current_time - nav->timestamp;
    return isfinite(age) && age >= 0.0 && age <= 2.0;
}