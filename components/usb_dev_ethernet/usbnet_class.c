/**
 * @file usbnet_class.c
 * @brief Runtime class dispatcher (see usbnet_class.h). One static kind,
 *        set once before the USB stack starts; the data-path calls are a
 *        two-way branch, not a vtable — there are exactly two classes and
 *        the branch predicts perfectly.
 */
#include "private/usbnet_class.h"

#include "usbd_core.h" /* defines __PACKED etc. — must precede class hdrs */
#include "usbd_cdc_ncm.h"
#include "usbd_rndis.h"

static usbnet_class_kind_t s_kind = USBNET_CLASS_KIND_NCM;

void usbnet_class_select(usbnet_class_kind_t kind)
{
    s_kind = kind;
}

usbnet_class_kind_t usbnet_class_kind(void)
{
    return s_kind;
}

const char *usbnet_class_name(void)
{
    return (s_kind == USBNET_CLASS_KIND_RNDIS) ? "RNDIS" : "CDC-NCM";
}

struct pbuf *usbnet_class_eth_rx(void)
{
    return (s_kind == USBNET_CLASS_KIND_RNDIS) ? usbd_rndis_eth_rx()
                                               : usbnet_ncm_eth_rx();
}

int usbnet_class_eth_tx(struct pbuf *p)
{
    return (s_kind == USBNET_CLASS_KIND_RNDIS) ? usbd_rndis_eth_tx(p)
                                               : usbnet_ncm_eth_tx(p);
}

void usbnet_class_set_connect(bool connect)
{
    if (s_kind == USBNET_CLASS_KIND_RNDIS)
    {
        (void)usbd_rndis_set_connect(connect);
    }

    /* NCM: link state is host-driven (data-interface alt setting); the
     * class raises usbd_cdc_ncm_link_event() — nothing to assert here. */
}
