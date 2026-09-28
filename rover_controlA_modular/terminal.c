#define _POSIX_C_SOURCE 200809L

#include "terminal.h"

#include <fcntl.h>
#include <unistd.h>

void terminal_start(struct Terminal *terminal)
{
    struct termios settings;

    terminal->settings_saved = 0;
    terminal->flags_saved = 0;

    if (tcgetattr(STDIN_FILENO, &terminal->original_settings) == 0) {
        terminal->settings_saved = 1;
        settings = terminal->original_settings;
        settings.c_lflag &= (tcflag_t)~(ICANON | ECHO);
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        (void)tcsetattr(STDIN_FILENO, TCSANOW, &settings);
    }

    terminal->original_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (terminal->original_flags >= 0) {
        terminal->flags_saved = 1;
        (void)fcntl(STDIN_FILENO, F_SETFL,
                    terminal->original_flags | O_NONBLOCK);
    }
}

int terminal_read_key(char *key)
{
    ssize_t count = read(STDIN_FILENO, key, 1);

    if (count <= 0)
        return 0;
    if (*key >= 'A' && *key <= 'Z')
        *key = (char)(*key - 'A' + 'a');
    return 1;
}

void terminal_restore(const struct Terminal *terminal)
{
    if (terminal->settings_saved)
        (void)tcsetattr(STDIN_FILENO, TCSANOW,
                        &terminal->original_settings);
    if (terminal->flags_saved)
        (void)fcntl(STDIN_FILENO, F_SETFL, terminal->original_flags);
}