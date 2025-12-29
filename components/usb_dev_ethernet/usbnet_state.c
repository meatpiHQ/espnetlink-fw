#include "private/usbnet_state.h"

static usbnet_state_t s_state;

usbnet_state_t *usbnet_state(void)
{
    return &s_state;
}
