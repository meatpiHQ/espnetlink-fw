/**
 * @file usbnet_ncm.c
 * @brief CDC-NCM ↔ raw-lwip pbuf glue. The WiCAN-authored usbd_cdc_ncm
 *        class (vendored cherryusb fork) exposes NTB-level APIs — unlike
 *        RNDIS/ECM it has no built-in lwIP eth_rx/eth_tx — so this file
 *        adapts it to the same pbuf contract usbnet_lwip.c consumes:
 *        eth_rx() returns one frame-pbuf per call (NULL when drained),
 *        eth_tx(p) returns 0 / -USB_ERR_BUSY (caller retries after
 *        tx-done) / other negative (drop).
 *
 *        RX: the ISR hook only latches the NTB length and wakes the rx
 *        task; parsing (usbd_cdc_ncm_parse_ntb → per-datagram pbufs into
 *        a small ring) happens in tcpip-task context inside eth_rx(),
 *        which then re-arms the read. TX: the pbuf chain is flattened to
 *        a staging frame and usbd_cdc_ncm_eth_start_write() builds the
 *        NTB16 around it.
 */
#include "sdkconfig.h"

#if CONFIG_USBNET_CLASS_NCM

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "lwip/pbuf.h"

#include "usbd_core.h" /* defines __PACKED etc. — must precede class hdrs */
#include "usbd_cdc_ncm.h"
#include "usb_errno.h"

#include "private/usbnet_usb.h"

static const char *TAG = "usb_dev_ethernet";

#define NCM_BUF_CAP 2048 /* one NTB each way (class default NTB16 max) */
#define NCM_RXQ_LEN 8    /* datagrams per NTB is small at FS sizes */
#define NCM_MAX_SEG 1514

static uint8_t *s_rx_ntb; /* INTERNAL|DMA — USB endpoint buffers */
static uint8_t *s_tx_ntb;
static uint8_t s_frame[NCM_MAX_SEG]; /* TX flatten staging (tcpip ctx) */

static volatile uint32_t s_pending_len;
static struct pbuf *s_rxq[NCM_RXQ_LEN];
static volatile uint8_t s_rq_head;
static volatile uint8_t s_rq_tail;

/* ---- class hooks (USB ISR context on the ESP port) ---------------------- */

void usbd_cdc_ncm_data_recv_done(uint32_t len)
{
    s_pending_len = len;
    usbnet_usb_notify_rx();
}

void usbd_cdc_ncm_data_send_done(uint32_t len)
{
    (void)len;
    usbnet_usb_notify_tx_done();
}

/* NCM link = the host selecting data-interface alt 1 (SET_INTERFACE) —
 * unlike RNDIS/ECM there is no device-side set_connect. */
void usbd_cdc_ncm_link_event(bool up)
{
    usbnet_usb_notify_link(up);
}

/* ---- lifecycle (evt task context) --------------------------------------- */

void usbnet_ncm_start(void)
{
    if (s_rx_ntb == NULL)
    {
        s_rx_ntb = heap_caps_malloc(NCM_BUF_CAP,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        s_tx_ntb = heap_caps_malloc(NCM_BUF_CAP,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (s_rx_ntb == NULL || s_tx_ntb == NULL)
        {
            ESP_LOGE(TAG, "NCM NTB buffer alloc failed");
            return;
        }
    }

    s_pending_len = 0;

    int ret = usbd_cdc_ncm_start_read(s_rx_ntb, NCM_BUF_CAP);

    if (ret != 0)
    {
        ESP_LOGW(TAG, "NCM initial read arm failed (%d)", ret);
    }
}

void usbnet_ncm_stop(void)
{
    s_pending_len = 0;

    while (s_rq_tail != s_rq_head)
    {
        pbuf_free(s_rxq[s_rq_tail]);
        s_rq_tail = (uint8_t)((s_rq_tail + 1) % NCM_RXQ_LEN);
    }
}

/* ---- pbuf adapters (tcpip task context) ---------------------------------- */

static void ncm_deliver(const uint8_t *frame, uint16_t len, void *arg)
{
    (void)arg;

    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (p == NULL)
    {
        return;
    }

    if (pbuf_take(p, frame, len) != ERR_OK)
    {
        pbuf_free(p);
        return;
    }

    uint8_t next = (uint8_t)((s_rq_head + 1) % NCM_RXQ_LEN);

    if (next == s_rq_tail)
    {
        pbuf_free(p); /* ring full — drop */
        return;
    }

    s_rxq[s_rq_head] = p;
    s_rq_head = next;
}

struct pbuf *usbnet_ncm_eth_rx(void)
{
    if (s_rq_tail == s_rq_head)
    {
        uint32_t len = s_pending_len;

        if (len == 0 || s_rx_ntb == NULL)
        {
            return NULL;
        }

        s_pending_len = 0;

        if (usbd_cdc_ncm_parse_ntb(s_rx_ntb, len, ncm_deliver, NULL) < 0)
        {
            ESP_LOGD(TAG, "NTB parse failed (len %u)", (unsigned)len);
        }

        (void)usbd_cdc_ncm_start_read(s_rx_ntb, NCM_BUF_CAP);

        if (s_rq_tail == s_rq_head)
        {
            return NULL;
        }
    }

    struct pbuf *p = s_rxq[s_rq_tail];

    s_rq_tail = (uint8_t)((s_rq_tail + 1) % NCM_RXQ_LEN);
    return p;
}

int usbnet_ncm_eth_tx(struct pbuf *p)
{
    if (s_tx_ntb == NULL)
    {
        return -USB_ERR_NOTCONN;
    }

    if (p->tot_len > sizeof(s_frame))
    {
        return -USB_ERR_RANGE;
    }

    (void)pbuf_copy_partial(p, s_frame, p->tot_len, 0);

    /* the class returns the NTB total (positive) on success — the
     * usbnet_lwip contract wants 0 */
    int ret = usbd_cdc_ncm_eth_start_write(s_tx_ntb, NCM_BUF_CAP,
                                           s_frame, p->tot_len);

    return (ret < 0) ? ret : 0;
}

#endif /* CONFIG_USBNET_CLASS_NCM */
