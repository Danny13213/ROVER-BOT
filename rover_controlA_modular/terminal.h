#ifndef TERMINAL_H
#define TERMINAL_H

#include <termios.h>

struct Terminal {
    struct termios original_settings;
    int settings_saved;
    int original_flags;
    int flags_saved;
};

void terminal_start(struct Terminal *terminal);
int terminal_read_key(char *key);
void terminal_restore(const struct Terminal *terminal);

#endif