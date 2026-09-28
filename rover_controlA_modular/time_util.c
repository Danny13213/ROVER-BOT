#define _POSIX_C_SOURCE 200809L

#include "time_util.h"

#include <time.h>

long long monotonic_milliseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

double realtime_seconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}

void sleep_milliseconds(long milliseconds)
{
    struct timespec delay = {
        .tv_sec = milliseconds / 1000,
        .tv_nsec = (milliseconds % 1000) * 1000000L
    };
    nanosleep(&delay, 0);
}