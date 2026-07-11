/**
 * @file cmdline_manager_line.c
 * @brief PURE byte-stream -> line assembler — no RTOS; host-testable.
 */
#include <string.h>

#include "cmdline_manager_private.h"

void cm_line_reset(cm_line_asm_t *a)
{
    a->len = 0;
    a->overflow = false;
}

size_t cm_line_feed(cm_line_asm_t *a, const uint8_t *data, size_t len,
                    const char **out_line)
{
    *out_line = NULL;

    for (size_t i = 0; i < len; i++)
    {
        char c = (char)data[i];

        if (c == '\n' || c == '\r')
        {
            bool had_overflow = a->overflow;
            size_t line_len = a->len;

            a->buf[line_len] = '\0';
            a->len = 0;
            a->overflow = false;

            if (line_len == 0 || had_overflow)
            {
                continue; /* empty (or the CRLF tail) / oversized: skip */
            }

            *out_line = a->buf;
            return i + 1;
        }

        if (a->len < sizeof(a->buf) - 1)
        {
            a->buf[a->len++] = c;
        }
        else
        {
            a->overflow = true; /* discard up to the terminator */
        }
    }

    return len;
}
