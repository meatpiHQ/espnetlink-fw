/**
 * @file vpn_manager_events.c
 * @brief event_manager glue: the `vpn.state {connected}` source —
 *        rules can alert on tunnel up/down or nudge consumers that
 *        live behind the tunnel. Publishes come from the state task
 *        (queue-safe).
 */
#include <stdio.h>

#include "event_manager.h"

#include "vpn_manager_private.h"

void vpn_events_register(void)
{
    static const em_key_decl_t KEYS[] =
    {
        { "connected", EM_VAL_STR },
    };
    static const em_source_decl_t STATE =
    {
        .source = "vpn", .name = "state",
        .description = "WireGuard tunnel state changed; {connected} is "
                       "\"true\" after a verified handshake, \"false\" "
                       "on teardown",
        .keys = KEYS, .n_keys = 1,
    };

    (void)event_manager_declare_source(&STATE);
}

void vpn_events_state(bool connected)
{
    em_event_t ev = { 0 };

    snprintf(ev.source, sizeof(ev.source), "vpn");
    snprintf(ev.name, sizeof(ev.name), "state");
    ev.kv[0] = em_kv_str("connected", connected ? "true" : "false");
    ev.n = 1;
    (void)event_manager_publish(&ev);
}
