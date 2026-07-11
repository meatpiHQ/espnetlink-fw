/**
 * @file http_client_manager_events.c
 * @brief event_manager glue. ESPNetLink does not compose an event_manager,
 *        so this is a no-op kept only to preserve the shared core's file
 *        shape (WiCAN registers the `http.post {url, body, content_type?}`
 *        action from here).
 */
#include "http_client_manager_private.h"

void hcm_events_register(void)
{
}
