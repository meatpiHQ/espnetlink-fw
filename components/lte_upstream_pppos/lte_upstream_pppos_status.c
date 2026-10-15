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
 *        Also the pure parsers of the modem's AT+QPINC, AT+CBC and AT+QTEMP
 *        answers (2026-10-13/14; the last two feed the production test).
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

/* The modem's answer to AT+CBC: `+CBC: <bcs>,<bcl>,<voltage>`; the voltage
 * (mV) is the LAST integer on the line, a unit suffix ("mV") may follow. */
bool lte_upstream_pppos_cbc_mv(const char *resp, int *mv)
{
    if (resp == NULL)
    {
        return false;
    }

    const char *p = strstr(resp, "+CBC:");

    if (p == NULL)
    {
        return false;
    }
    p += strlen("+CBC:");

    long value = -1;

    while (*p != '\0' && *p != '\r' && *p != '\n')
    {
        while (*p == ' ' || *p == ',')
        {
            p++;
        }

        char *end = NULL;
        long v = strtol(p, &end, 10);

        if (end == p)
        {
            break;
        }
        value = v;
        p = end;

        if (*p != ',' && *p != ' ')
        {
            break; /* end of line, or a unit suffix */
        }
    }

    /* a modem runs between 3.3 and 4.3 V; anything else is not a reading */
    if (value < 1000 || value > 60000)
    {
        return false;
    }
    if (mv != NULL)
    {
        *mv = (int)value;
    }
    return true;
}

/* The modem's answer to AT+QTEMP, in either spelling: every pure-integer
 * field after "+QTEMP:" (quoted or not) is a temperature, sensor names
 * such as "qfe_wtr_pa0" are not integers and fall out; the hottest wins. */
bool lte_upstream_pppos_qtemp_c(const char *resp, int *temp_c)
{
    if (resp == NULL)
    {
        return false;
    }

    const char *p = strstr(resp, "+QTEMP:");

    if (p == NULL)
    {
        return false;
    }

    bool found = false;
    long best = 0;

    while (*p != '\0')
    {
        while (*p == ' ' || *p == ',' || *p == '"' || *p == '\r' ||
               *p == '\n' || *p == '\t')
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }

        const char *tok = p;

        while (*p != '\0' && *p != ' ' && *p != ',' && *p != '"' &&
               *p != '\r' && *p != '\n' && *p != '\t')
        {
            p++;
        }

        size_t len = (size_t)(p - tok);
        size_t i = (tok[0] == '-') ? 1 : 0;
        bool is_int = (len > i);

        for (size_t k = i; k < len && is_int; k++)
        {
            is_int = (tok[k] >= '0' && tok[k] <= '9');
        }

        if (is_int)
        {
            long v = strtol(tok, NULL, 10);

            /* a sensor reading, not a counter: -40..125 C */
            if (v >= -40 && v <= 125 && (!found || v > best))
            {
                best = v;
                found = true;
            }
        }
    }

    if (!found)
    {
        return false;
    }
    if (temp_c != NULL)
    {
        *temp_c = (int)best;
    }
    return true;
}
