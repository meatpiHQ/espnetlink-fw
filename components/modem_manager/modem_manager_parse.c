/*
 * This file is part of the ESPNetLink project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file modem_manager_parse.c
 * @brief Pure AT-response parsers (moved verbatim from modem_manager_probe.c
 *        so they host-test on the linux target, standard §7).
 */

#include <stdio.h>

#include "modem_manager_parse.h"

bool mm_parse_ifc(const char *s, int *rx, int *tx)
{
    // formats: "2,2"
    int r = -1, t = -1;
    if (sscanf(s, "%d,%d", &r, &t) == 2)
    {
        *rx = r; *tx = t;
        return true;
    }
    return false;
}

bool mm_parse_ipr(const char *s, int *baud)
{
    int b = -1;
    if (sscanf(s, "%d", &b) == 1)
    {
        *baud = b;
        return true;
    }
    return false;
}
