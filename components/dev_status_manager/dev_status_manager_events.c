/**
 * @file dev_status_manager_events.c
 * @brief event_manager glue. ESPNetLink does not compose an event_manager,
 *        so this is a no-op kept only to preserve the shared core's file
 *        shape (WiCAN publishes `status.bit` events from here).
 */
#include "dev_status_manager.h"

void dsm_events_register(void)
{
}
