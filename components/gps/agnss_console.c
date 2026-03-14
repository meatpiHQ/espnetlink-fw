/*
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/**
 * @file agnss_console.c
 * @brief CLI command for AGNSS status and control.
 *
 * Provides the `agnss` console command:
 *   agnss status   — show EASY status and cached position
 *   agnss save     — save EASY navigation data
 *   agnss enable   — enable AGNSS (persists to config)
 *   agnss disable  — disable AGNSS (persists to config)
 */

#include "gps.h"
#include "agnss_internal.h"

#include <stdio.h>
#include <string.h>

#include "esp_console.h"
#include "config_manager.h"

static bool s_registered = false;

static int cmd_agnss(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: agnss -s | -v | -e | -d\n");
        printf("  -s  status    Show AGNSS status\n");
        printf("  -v  save      Save EASY navigation data\n");
        printf("  -e  enable    Enable AGNSS (next boot)\n");
        printf("  -d  disable   Disable AGNSS (next boot)\n");
        return 1;
    }

    const char *sub = argv[1];

    if (strcmp(sub, "-s") == 0 || strcmp(sub, "status") == 0)
    {
        bool enabled = true;
        config_get_bool("AGNSS_ENABLED", &enabled);
        printf("AGNSS: %s\n", enabled ? "enabled" : "disabled");

        bool time_ok = false, pos_ok = false;
        agnss_get_status(&time_ok, &pos_ok);
        printf("Time injected:     %s\n", time_ok ? "yes" : "no");
        printf("Position injected: %s\n", pos_ok ? "yes" : "no");

        double lat, lon, alt;
        if (agnss_nvs_read_position(&lat, &lon, &alt) == ESP_OK)
        {
            printf("Cached position: lat=%.6f lon=%.6f alt=%.1f m\n",
                   lat, lon, alt);
        }
        else
        {
            printf("No cached position in NVS.\n");
        }
        return 0;
    }

    if (strcmp(sub, "-v") == 0 || strcmp(sub, "save") == 0)
    {
        esp_err_t err = agnss_save_easy_data();
        if (err == ESP_OK)
        {
            printf("EASY data saved.\n");
        }
        else
        {
            printf("EASY save failed: %s\n", esp_err_to_name(err));
        }
        return (err == ESP_OK) ? 0 : 1;
    }

    if (strcmp(sub, "-e") == 0 || strcmp(sub, "enable") == 0)
    {
        config_set_bool("AGNSS_ENABLED", true);
        config_save();
        printf("AGNSS enabled (takes effect on next boot).\n");
        return 0;
    }

    if (strcmp(sub, "-d") == 0 || strcmp(sub, "disable") == 0)
    {
        config_set_bool("AGNSS_ENABLED", false);
        config_save();
        printf("AGNSS disabled (takes effect on next boot).\n");
        return 0;
    }

    printf("Unknown option: %s\n", sub);
    printf("Usage: agnss -s | -v | -e | -d\n");
    return 1;
}

void agnss_console_register(void)
{
    if (s_registered)
    {
        return;
    }

    const esp_console_cmd_t cmd = {
        .command = "agnss",
        .help = "AGNSS helpers: -s status, -v save EASY, -e enable, -d disable",
        .hint = NULL,
        .func = &cmd_agnss,
    };

    (void)esp_console_cmd_register(&cmd);
    s_registered = true;
}
