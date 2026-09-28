#ifndef TIME_UTIL_H
#define TIME_UTIL_H

long long monotonic_milliseconds(void);
double realtime_seconds(void);
void sleep_milliseconds(long milliseconds);

#endif