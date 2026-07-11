/* Host-test stand-in for modem_manager.h (shadowed via include order):
 * lte_upstream_pppos.h embeds modem_mgr_config_t by value, so the type just
 * has to be complete — nothing in the settings TU touches its members. */
#pragma once

typedef struct
{
    int host_test_placeholder;
} modem_mgr_config_t;
