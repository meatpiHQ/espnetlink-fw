/* Host-test stand-in for esp_netif.h (shadowed via include order):
 * lte_upstream_pppos.h only needs esp_ip4_addr_t from it. */
#pragma once

#include <stdint.h>

typedef struct
{
    uint32_t addr;
} esp_ip4_addr_t;
