/*
 * usbd_cdc_ncm.c — CDC-NCM (NTB16) DEVICE class.
 *
 * WiCAN-AUTHORED (2026-07-07, see PROVENANCE.md and usbd_cdc_ncm.h).
 * Style/structure follows usbd_cdc_ecm.c and usbd_rndis.c.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usbd_cdc_ncm.h"

#define CDC_NCM_OUT_EP_IDX 0
#define CDC_NCM_IN_EP_IDX  1
#define CDC_NCM_INT_EP_IDX 2

/* interrupt-EP notification chain after the host opens the data path:
 * ConnectionSpeedChange first, then NetworkConnection (NCM 1.0 §7.1) */
#define NCM_NOTIFY_IDLE       0
#define NCM_NOTIFY_SPEED_SENT 1
#define NCM_NOTIFY_DONE       2

static struct usbd_endpoint cdc_ncm_ep_data[3];

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_ncm_notify_buf[USB_ALIGN_UP(16, CONFIG_USB_ALIGN_SIZE)];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_ncm_ctrl_buf[USB_ALIGN_UP(sizeof(struct cdc_ncm_ntb_parameters), CONFIG_USB_ALIGN_SIZE)];

static struct usbd_interface *g_comm_intf;
static volatile uint8_t g_data_alt;
static volatile uint8_t g_notify_state;
static volatile uint32_t g_ncm_rx_data_length;
static volatile uint32_t g_ncm_tx_data_length;
static uint32_t g_ntb_in_max_size = CONFIG_USBDEV_CDC_NCM_MAX_NTB_SIZE;
static uint16_t g_tx_sequence;

static void cdc_ncm_send_notify(uint8_t notifycode)
{
    struct cdc_eth_notification *notify =
        (struct cdc_eth_notification *)g_ncm_notify_buf;
    uint8_t bytes2send = 0;

    notify->bmRequestType = CDC_ECM_BMREQUEST_TYPE_ECM;
    notify->bNotificationType = notifycode;
    notify->wIndex = g_comm_intf ? g_comm_intf->intf_num : 0;
    memset(notify->data, 0, sizeof(notify->data));

    switch (notifycode) {
        case CDC_ECM_NOTIFY_CODE_NETWORK_CONNECTION:
            notify->wValue = CDC_ECM_NET_CONNECTED;
            notify->wLength = 0U;
            bytes2send = 8U;
            break;
        case CDC_ECM_NOTIFY_CODE_CONNECTION_SPEED_CHANGE: {
            uint32_t speed;

            if (usbd_get_ep_mps(0, cdc_ncm_ep_data[CDC_NCM_IN_EP_IDX].ep_addr) > 64) {
                speed = 480000000U;
            } else {
                speed = 12000000U;
            }
            notify->wValue = 0U;
            notify->wLength = 8U;
            memcpy(&notify->data[0], &speed, 4); /* upstream */
            memcpy(&notify->data[4], &speed, 4); /* downstream */
            bytes2send = 16U;
            break;
        }
        default:
            break;
    }

    if (bytes2send && usb_device_is_configured(0)) {
        usbd_ep_start_write(0, cdc_ncm_ep_data[CDC_NCM_INT_EP_IDX].ep_addr,
                            g_ncm_notify_buf, bytes2send);
    }
}

static int cdc_ncm_class_interface_request_handler(uint8_t busid,
                                                   struct usb_setup_packet *setup,
                                                   uint8_t **data, uint32_t *len)
{
    (void)busid;

    switch (setup->bRequest) {
        case CDC_REQUEST_GET_NTB_PARAMETERS: {
            struct cdc_ncm_ntb_parameters *param =
                (struct cdc_ncm_ntb_parameters *)g_ncm_ctrl_buf;

            memset(param, 0, sizeof(*param));
            param->wLength = sizeof(*param);
            param->bmNtbFormatsSupported = 0x0001; /* NTB16 only */
            param->dwNtbInMaxSize = CONFIG_USBDEV_CDC_NCM_MAX_NTB_SIZE;
            param->wNdbInDivisor = 4;
            param->wNdbInPayloadRemainder = 0;
            param->wNdbInAlignment = 4;
            param->dwNtbOutMaxSize = CONFIG_USBDEV_CDC_NCM_MAX_NTB_SIZE;
            param->wNdbOutDivisor = 4;
            param->wNdbOutPayloadRemainder = 0;
            param->wNdbOutAlignment = 4;
            param->wNtbOutMaxDatagrams = 8;

            *data = g_ncm_ctrl_buf;
            *len = sizeof(*param);
            break;
        }
        case CDC_REQUEST_GET_NTB_INPUT_SIZE:
            memcpy(g_ncm_ctrl_buf, (const void *)&g_ntb_in_max_size, 4);
            *data = g_ncm_ctrl_buf;
            *len = 4;
            break;
        case CDC_REQUEST_SET_NTB_INPUT_SIZE: {
            uint32_t sz;

            if (setup->wLength < 4) {
                return -1;
            }
            memcpy(&sz, *data, 4);
            /* 2048 is the spec minimum; we never build bigger NTBs than
             * CONFIG_USBDEV_CDC_NCM_MAX_NTB_SIZE anyway */
            if (sz < 2048U) {
                return -1;
            }
            g_ntb_in_max_size = sz;
            break;
        }
        case CDC_REQUEST_GET_NTB_FORMAT:
            g_ncm_ctrl_buf[0] = 0x00; /* NTB16 */
            g_ncm_ctrl_buf[1] = 0x00;
            *data = g_ncm_ctrl_buf;
            *len = 2;
            break;
        case CDC_REQUEST_SET_NTB_FORMAT:
            if (setup->wValue != 0x0000) {
                return -1; /* NTB32 unsupported */
            }
            break;
        case CDC_REQUEST_SET_ETHERNET_PACKET_FILTER:
            /* we deliver everything upstream; accept any filter */
            break;
        default:
            USB_LOG_WRN("Unhandled CDC NCM bRequest 0x%02x\r\n",
                        setup->bRequest);
            return -1;
    }

    return 0;
}

static void cdc_ncm_notify_handler(uint8_t busid, uint8_t event, void *arg)
{
    (void)busid;

    switch (event) {
        case USBD_EVENT_RESET:
            g_data_alt = 0;
            g_notify_state = NCM_NOTIFY_IDLE;
            g_ncm_rx_data_length = 0;
            g_ncm_tx_data_length = 0;
            g_tx_sequence = 0;
            g_ntb_in_max_size = CONFIG_USBDEV_CDC_NCM_MAX_NTB_SIZE;
            usbd_cdc_ncm_link_event(false);
            break;
        case USBD_EVENT_SET_INTERFACE: {
            struct usb_interface_descriptor *desc =
                (struct usb_interface_descriptor *)arg;

            /* only the DATA interface has alternate settings */
            if (desc == NULL ||
                (g_comm_intf && desc->bInterfaceNumber == g_comm_intf->intf_num)) {
                break;
            }

            if (desc->bAlternateSetting == 1 && g_data_alt != 1) {
                g_data_alt = 1;
                g_ncm_tx_data_length = 0;
                g_tx_sequence = 0;
                g_notify_state = NCM_NOTIFY_IDLE;
                cdc_ncm_send_notify(CDC_ECM_NOTIFY_CODE_CONNECTION_SPEED_CHANGE);
                g_notify_state = NCM_NOTIFY_SPEED_SENT;
                usbd_cdc_ncm_link_event(true);
            } else if (desc->bAlternateSetting == 0 && g_data_alt != 0) {
                g_data_alt = 0;
                usbd_cdc_ncm_link_event(false);
            }
            break;
        }
        default:
            break;
    }
}

static void cdc_ncm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;

    g_ncm_rx_data_length = nbytes;
    usbd_cdc_ncm_data_recv_done(nbytes);
}

static void cdc_ncm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;

    if ((nbytes % usbd_get_ep_mps(0, ep)) == 0 && nbytes) {
        /* the NTB length is a multiple of MPS: terminate with a ZLP so the
         * host's read of dwNtbInMaxSize completes immediately */
        usbd_ep_start_write(0, ep, NULL, 0);
    } else {
        uint32_t len = g_ncm_tx_data_length;

        g_ncm_tx_data_length = 0;
        usbd_cdc_ncm_data_send_done(len);
    }
}

static void cdc_ncm_int_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;

    if (g_notify_state == NCM_NOTIFY_SPEED_SENT) {
        g_notify_state = NCM_NOTIFY_DONE;
        cdc_ncm_send_notify(CDC_ECM_NOTIFY_CODE_NETWORK_CONNECTION);
    }
}

bool usbd_cdc_ncm_data_ready(void)
{
    return usb_device_is_configured(0) && (g_data_alt == 1);
}

int usbd_cdc_ncm_start_read(uint8_t *buf, uint32_t len)
{
    if (!usbd_cdc_ncm_data_ready()) {
        return -USB_ERR_NOTCONN;
    }

    g_ncm_rx_data_length = 0;
    return usbd_ep_start_read(0, cdc_ncm_ep_data[CDC_NCM_OUT_EP_IDX].ep_addr,
                              buf, len);
}

int usbd_cdc_ncm_start_write(uint8_t *buf, uint32_t len)
{
    if (!usbd_cdc_ncm_data_ready()) {
        return -USB_ERR_NOTCONN;
    }

    if (g_ncm_tx_data_length > 0) {
        return -USB_ERR_BUSY;
    }

    g_ncm_tx_data_length = len;
    return usbd_ep_start_write(0, cdc_ncm_ep_data[CDC_NCM_IN_EP_IDX].ep_addr,
                               buf, len);
}

int usbd_cdc_ncm_eth_start_write(uint8_t *ntb_buf, uint32_t cap,
                                 const uint8_t *frame, uint32_t frame_len)
{
    /* NTH16 (12) | NDP16 with 1 datagram + terminator (16) | frame @28.
     * 28 satisfies the divisor-4 alignment we advertise. */
    struct cdc_ncm_nth16 *nth = (struct cdc_ncm_nth16 *)ntb_buf;
    struct cdc_ncm_ndp16 *ndp = (struct cdc_ncm_ndp16 *)(ntb_buf + 12);
    uint32_t total = 28U + frame_len;
    int ret;

    if (total > cap || total > g_ntb_in_max_size) {
        return -USB_ERR_RANGE;
    }

    if (g_ncm_tx_data_length > 0) {
        return -USB_ERR_BUSY;
    }

    nth->dwSignature = CDC_NCM_NTH16_SIGNATURE;
    nth->wHeaderLength = 12;
    nth->wSequence = g_tx_sequence++;
    nth->wBlockLength = (uint16_t)total;
    nth->wNdpIndex = 12;

    ndp->dwSignature = CDC_NCM_NDP16_SIGNATURE_NCM0;
    ndp->wLength = 16;
    ndp->wNextNdpIndex = 0;
    ndp->datagram[0].wDatagramIndex = 28;
    ndp->datagram[0].wDatagramLength = (uint16_t)frame_len;
    ndp->datagram[1].wDatagramIndex = 0; /* terminator */
    ndp->datagram[1].wDatagramLength = 0;

    memcpy(ntb_buf + 28, frame, frame_len);

    ret = usbd_cdc_ncm_start_write(ntb_buf, total);
    return (ret < 0) ? ret : (int)total;
}

int usbd_cdc_ncm_parse_ntb(const uint8_t *ntb, uint32_t len,
                           usbd_cdc_ncm_datagram_cb_t cb, void *arg)
{
    const struct cdc_ncm_nth16 *nth = (const struct cdc_ncm_nth16 *)ntb;
    uint32_t ndp_off;
    int count = 0;

    if (len < 12 || nth->dwSignature != CDC_NCM_NTH16_SIGNATURE ||
        nth->wHeaderLength < 12 || nth->wBlockLength > len) {
        return -1;
    }

    ndp_off = nth->wNdpIndex;

    while (ndp_off != 0) {
        const struct cdc_ncm_ndp16 *ndp;
        uint32_t entries;

        if (ndp_off + 8 > len) {
            return -1;
        }
        ndp = (const struct cdc_ncm_ndp16 *)(ntb + ndp_off);
        if ((ndp->dwSignature != CDC_NCM_NDP16_SIGNATURE_NCM0 &&
             ndp->dwSignature != CDC_NCM_NDP16_SIGNATURE_NCM1) ||
            ndp->wLength < 16 || ndp_off + ndp->wLength > len) {
            return -1;
        }

        entries = (ndp->wLength - 8U) / 4U;

        for (uint32_t i = 0; i < entries; i++) {
            uint16_t d_off = ndp->datagram[i].wDatagramIndex;
            uint16_t d_len = ndp->datagram[i].wDatagramLength;

            if (d_off == 0 || d_len == 0) {
                break; /* terminator */
            }
            if ((uint32_t)d_off + d_len > len) {
                return -1;
            }
            cb(ntb + d_off, d_len, arg);
            count++;
        }

        ndp_off = ndp->wNextNdpIndex;
    }

    return count;
}

struct usbd_interface *usbd_cdc_ncm_init_intf(struct usbd_interface *intf,
                                              const uint8_t int_ep,
                                              const uint8_t out_ep,
                                              const uint8_t in_ep)
{
    intf->class_interface_handler = cdc_ncm_class_interface_request_handler;
    intf->class_endpoint_handler = NULL;
    intf->vendor_handler = NULL;
    intf->notify_handler = cdc_ncm_notify_handler;

    g_comm_intf = intf;

    cdc_ncm_ep_data[CDC_NCM_OUT_EP_IDX].ep_addr = out_ep;
    cdc_ncm_ep_data[CDC_NCM_OUT_EP_IDX].ep_cb = cdc_ncm_bulk_out;
    cdc_ncm_ep_data[CDC_NCM_IN_EP_IDX].ep_addr = in_ep;
    cdc_ncm_ep_data[CDC_NCM_IN_EP_IDX].ep_cb = cdc_ncm_bulk_in;
    cdc_ncm_ep_data[CDC_NCM_INT_EP_IDX].ep_addr = int_ep;
    cdc_ncm_ep_data[CDC_NCM_INT_EP_IDX].ep_cb = cdc_ncm_int_in;

    usbd_add_endpoint(0, &cdc_ncm_ep_data[CDC_NCM_OUT_EP_IDX]);
    usbd_add_endpoint(0, &cdc_ncm_ep_data[CDC_NCM_IN_EP_IDX]);
    usbd_add_endpoint(0, &cdc_ncm_ep_data[CDC_NCM_INT_EP_IDX]);

    return intf;
}

struct usbd_interface *usbd_cdc_ncm_init_data_intf(struct usbd_interface *intf)
{
    intf->class_interface_handler = NULL;
    intf->class_endpoint_handler = NULL;
    intf->vendor_handler = NULL;
    /* alt-setting changes on the data interface arrive here */
    intf->notify_handler = cdc_ncm_notify_handler;

    return intf;
}

__WEAK void usbd_cdc_ncm_data_recv_done(uint32_t len)
{
    (void)len;
}

__WEAK void usbd_cdc_ncm_data_send_done(uint32_t len)
{
    (void)len;
}

__WEAK void usbd_cdc_ncm_link_event(bool up)
{
    (void)up;
}
