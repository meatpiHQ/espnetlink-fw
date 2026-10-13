/*
 * This file is part of the ESPNetLink firmware.
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
 * @file lte_upstream_pppos_status.c
 * @brief The wire spellings of the SIM state and the bring-up stage: pure
 *        (no IDF), host-tested, shared by `GET /api/lte`, the
 *        `/api/wifi_modem` health and the `lte -j` console so the three
 *        never drift (2026-10-07, for the WiCAN's Quick Setup checks).
 */
#include "lte_upstream_pppos.h"

#include <stdlib.h>
#include <string.h>

const char *lte_upstream_pppos_sim_str(lte_sim_state_t s)
{
    switch (s)
    {
        case LTE_SIM_READY:     return "ready";
        case LTE_SIM_MISSING:   return "missing";
        case LTE_SIM_PIN:       return "pin";
        case LTE_SIM_PIN_WRONG: return "pin_wrong";
        default:                return "unknown";
    }
}

bool lte_upstream_pppos_qpinc_remaining(const char *resp, int *pin_left,
                                        int *puk_left)
{
    if (resp == NULL)
    {
        return false;
    }

    const char *p = strstr(resp, "+QPINC: \"SC\",");

    if (p == NULL)
    {
        return false;
    }
    p += strlen("+QPINC: \"SC\",");

    char *end = NULL;
    long pin = strtol(p, &end, 10);

    if (end == p || *end != ',')
    {
        return false;
    }
    p = end + 1;

    long puk = strtol(p, &end, 10);

    if (end == p || pin < 0 || puk < 0 || pin > 99 || puk > 99)
    {
        return false;
    }
    if (pin_left != NULL)
    {
        *pin_left = (int)pin;
    }
    if (puk_left != NULL)
    {
        *puk_left = (int)puk;
    }
    return true;
}

const char *lte_upstream_pppos_stage_from(bool started, bool connected,
                                          bool disconnected, bool detached)
{
    if (!started)
    {
        return "not_started";
    }
    if (connected && !disconnected)
    {
        return "connected";
    }
    if (disconnected)
    {
        return "ppp_disconnected";
    }
    if (detached)
    {
        return "network_detached";
    }
    return "connecting";
}
