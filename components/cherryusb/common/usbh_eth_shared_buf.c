#include <stdbool.h>
#include "usb_config.h"
#include "usb_util.h"
#include "usb_log.h"
#include "usb_osal.h"
#include "usbh_eth_shared_buf.h"

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

/*
 * WiCAN change (meatpiHQ dynamic-shared-buffer, extended 2026-07-07):
 * the shared eth pool is HEAP-BACKED and refcounted — allocated from
 * internal DMA-capable RAM on the first eth-class connect, freed on
 * the last disconnect, so the ~(RX+TX+CTRL+INT) bytes are only
 * consumed while a USB-ethernet adapter is actually attached.
 * Internal-RAM budget: see wican-fw ARCHITECTURE §12b.
 */

uint8_t *g_usbh_eth_shared_rx_buffer;
uint8_t *g_usbh_eth_shared_tx_buffer;
uint8_t *g_usbh_eth_shared_int_buffer;
uint8_t *g_usbh_eth_shared_ctrl_buffer;

/* single-owner, not refcounted: this hardware runs ONE eth class at a
 * time (the shared pool's premise), and a connect that fails after
 * allocating never gets a disconnect — idempotent alloc + always-free
 * can't strand the pool the way a refcount can */
static bool s_allocated;

static uint8_t *eth_buf_alloc(size_t size)
{
#ifdef ESP_PLATFORM
    return heap_caps_aligned_alloc(CONFIG_USB_ALIGN_SIZE, size,
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
#else
    return usb_osal_malloc(size);
#endif
}

static void eth_buf_free(uint8_t *p)
{
    if (p == NULL) {
        return;
    }
#ifdef ESP_PLATFORM
    heap_caps_free(p);
#else
    usb_osal_free(p);
#endif
}

int usbh_eth_shared_buf_alloc(void)
{
    if (s_allocated) {
        return 0;
    }

    g_usbh_eth_shared_rx_buffer = eth_buf_alloc(USBH_ETH_SHARED_RX_SIZE);
    g_usbh_eth_shared_tx_buffer = eth_buf_alloc(USBH_ETH_SHARED_TX_SIZE);
    g_usbh_eth_shared_int_buffer = eth_buf_alloc(USBH_ETH_SHARED_INT_SIZE);
    g_usbh_eth_shared_ctrl_buffer = eth_buf_alloc(USBH_ETH_SHARED_CTRL_SIZE);

    if (g_usbh_eth_shared_rx_buffer == NULL ||
        g_usbh_eth_shared_tx_buffer == NULL ||
        g_usbh_eth_shared_int_buffer == NULL ||
        g_usbh_eth_shared_ctrl_buffer == NULL) {
        USB_LOG_ERR("eth shared buf alloc failed (rx %u + tx %u bytes)\r\n",
                    (unsigned)USBH_ETH_SHARED_RX_SIZE,
                    (unsigned)USBH_ETH_SHARED_TX_SIZE);
        usbh_eth_shared_buf_force_free();
        return -1;
    }

    s_allocated = true;
    USB_LOG_INFO("eth shared buf: %u B internal DMA\r\n",
                 (unsigned)(USBH_ETH_SHARED_RX_SIZE + USBH_ETH_SHARED_TX_SIZE +
                            USBH_ETH_SHARED_INT_SIZE + USBH_ETH_SHARED_CTRL_SIZE));
    return 0;
}

void usbh_eth_shared_buf_force_free(void)
{
    eth_buf_free(g_usbh_eth_shared_rx_buffer);
    eth_buf_free(g_usbh_eth_shared_tx_buffer);
    eth_buf_free(g_usbh_eth_shared_int_buffer);
    eth_buf_free(g_usbh_eth_shared_ctrl_buffer);
    g_usbh_eth_shared_rx_buffer = NULL;
    g_usbh_eth_shared_tx_buffer = NULL;
    g_usbh_eth_shared_int_buffer = NULL;
    g_usbh_eth_shared_ctrl_buffer = NULL;
    s_allocated = false;
}

void usbh_eth_shared_buf_free(void)
{
    usbh_eth_shared_buf_force_free();
}
