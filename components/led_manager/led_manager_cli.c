/*
 * This file is part of the MeatPi components project.
 *
 * Copyright (C) 2022-2026 MeatPi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file led_manager_cli.c
 * @brief The component's CLI command (`led`) — registered into
 *        cmdline_manager by led_manager_register_cli() (main wires it
 *        in CLI compositions only). Legacy option interface preserved
 *        (-i/--id, -c/--color <r> <g> <b>, -b/--blink); in v6 a manual
 *        color is an ALERT-priority indication (the arbiter owns the
 *        LED), so -x/--clear releases it; bare shows what's active.
 */
#include "argtable3/argtable3.h"

#include "cmdline_manager.h"

#include "led_manager.h"
#include "led_manager_private.h"

static struct
{
    struct arg_lit *id;
    struct arg_lit *color;
    struct arg_int *rgb;
    struct arg_lit *blink;
    struct arg_lit *clear;
    struct arg_lit *dump;
    struct arg_end *end;
} s_args;

static bool valid_u8(int v)
{
    return v >= 0 && v <= 255;
}

static int cmd_led(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&s_args);

    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_args.end, argv[0]);
        return 1;
    }

    if (s_args.id->count > 0)
    {
        uint8_t id;

        if (lm_aw2023_device_id(&id) != ESP_OK)
        {
            cmdline_printf("Error: Failed to read LED driver ID\n");
            return 1;
        }

        cmdline_printf("LED Driver ID: 0x%02X\n", id);
        cmdline_printf("OK\n");
        return 0;
    }

    if (s_args.color->count > 0)
    {
        if (s_args.rgb->count != 3)
        {
            cmdline_printf("Error: Missing RGB values. "
                           "Usage: led -c <r> <g> <b> [-b]\n");
            return 1;
        }

        int r = s_args.rgb->ival[0];
        int g = s_args.rgb->ival[1];
        int b = s_args.rgb->ival[2];

        if (!valid_u8(r) || !valid_u8(g) || !valid_u8(b))
        {
            cmdline_printf("Error: Color values must be in range 0-255\n");
            return 1;
        }

        led_manager_state_t state =
        {
            .mode = (s_args.blink->count > 0) ? LED_MANAGER_BLINK_FAST
                                              : LED_MANAGER_SOLID,
            .r = (uint8_t)r,
            .g = (uint8_t)g,
            .b = (uint8_t)b,
        };

        if (led_manager_set(LED_MANAGER_PRIO_ALERT, &state) != ESP_OK)
        {
            cmdline_printf("Error: Failed to set LED color\n");
            return 1;
        }

        cmdline_printf("OK\n"); /* shown unless OTA holds CRITICAL;
                                   release with led -x */
        return 0;
    }

    if (s_args.clear->count > 0)
    {
        led_manager_clear(LED_MANAGER_PRIO_ALERT);
        cmdline_printf("OK\n");
        return 0;
    }

    if (s_args.dump->count > 0)
    {
        /* AW2023 diag: GCR1.CHIPEN=0 after init means a UVLO/OTP trip
           dropped the chip to standby (LED writes silently ignored);
           ISR (0x02) names the cause but is clear-on-read. */
        static const struct { uint8_t reg; const char *name; } REGS[] =
        {
            { 0x01, "GCR1"  }, { 0x02, "ISR"   }, { 0x03, "PATST" },
            { 0x04, "GCR2"  }, { 0x30, "LCTR"  }, { 0x31, "LCFG0" },
            { 0x32, "LCFG1" }, { 0x33, "LCFG2" }, { 0x34, "PWM0"  },
            { 0x35, "PWM1"  }, { 0x36, "PWM2"  },
        };

        for (size_t i = 0; i < sizeof(REGS) / sizeof(REGS[0]); i++)
        {
            uint8_t v = 0;

            if (lm_aw2023_read_reg(REGS[i].reg, &v) != ESP_OK)
            {
                cmdline_printf("Error: read 0x%02X failed\n", REGS[i].reg);
                return 1;
            }

            cmdline_printf("%-5s (0x%02X) = 0x%02X\n", REGS[i].name,
                           REGS[i].reg, v);
        }

        cmdline_printf("OK\n");
        return 0;
    }

    /* bare: what the arbiter is showing right now */
    static const char *PRIO[] = { "idle", "status", "alert", "critical" };
    static const char *MODE[] = { "off", "solid", "blink_slow",
                                  "blink_fast" };
    led_manager_prio_t prio;
    led_manager_state_t st;

    if (led_manager_active(&prio, &st) != ESP_OK)
    {
        cmdline_printf("Error: LED unavailable\n");
        return 1;
    }

    cmdline_printf("LED: %s %s rgb(%u,%u,%u)\n", PRIO[prio],
                   MODE[st.mode], st.r, st.g, st.b);
    cmdline_printf("OK\n");
    return 0;
}

esp_err_t led_manager_register_cli(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "led",
        .help = "LED driver control",
        .hint = "Options: -i/--id, -c/--color <r> <g> <b>, -b/--blink, "
                "-x/--clear, -d/--dump",
        .func = cmd_led,
        .argtable = &s_args,
    };

    s_args.id = arg_lit0("i", "id", "Get LED driver device ID");
    s_args.color = arg_lit0("c", "color",
                            "Set LED RGB color using the following "
                            "<r> <g> <b>");
    s_args.rgb = arg_intn(NULL, NULL, "<r> <g> <b>", 0, 3,
                          "RGB values (0-255 each)");
    s_args.blink = arg_lit0("b", "blink",
                            "Blink the selected color channels");
    s_args.clear = arg_lit0("x", "clear",
                            "Release the manual color (back to normal)");
    s_args.dump = arg_lit0("d", "dump",
                           "Dump AW2023 control registers (diagnostics)");
    s_args.end = arg_end(6);
    return cmdline_manager_register(&CMD);
}
