#include "usb_dev_ethernet.h"

#include <stdlib.h>
#include <string.h>

#include <inttypes.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"

#include "tinyusb_net.h"

#include "dhserver.h"

#include "lwip/etharp.h"
#include "lwip/def.h"
#include "lwip/prot/ethernet.h"
#if CONFIG_LWIP_IPV4_NAPT
#include "lwip/lwip_napt.h"
#endif
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"

static const char *TAG = "usb_dev_eth";

// --------------------------------------------------------------------------------------
// Custom descriptors for Windows-friendly NCM enumeration
// - Use a different PID from the CDC bridge mode to avoid Windows driver caching issues.
// - Use bcdUSB = 0x0201 when running NCM.
// - Keep MAC string index compatible with esp_tinyusb tinyusb_net.c (uses tusb_get_mac_string_id()).
// --------------------------------------------------------------------------------------

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    // esp_tinyusb's tinyusb_net.c uses tusb_get_mac_string_id() from its usb_descriptors.c.
    // When CFG_TUD_CDC is enabled in tusb_config.h, it reserves one string ID for CDC,
    // so MAC ends up at index 6 (not 5). Keep our indices aligned to avoid Windows NCM driver failures.
    STRID_CDC_INTERFACE_DUMMY,
    STRID_NET,
    STRID_MAC,
};

static tusb_desc_device_t const s_dev_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0201,

    // Composite / IAD
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = 0x303A,
    .idProduct = 0x4006, // pick a PID distinct from the CDC bridge to avoid Windows caching
    .bcdDevice = 0x0100,

    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,

    .bNumConfigurations = 0x01,
};

// NCM uses 2 interfaces (control + data)
enum {
    ITF_NUM_NET = 0,
    ITF_NUM_NET_DATA,
    ITF_NUM_TOTAL,
};

#define EPNUM_NET_NOTIF 0x81
#define EPNUM_NET_OUT   0x02
#define EPNUM_NET_IN    0x82

#define NCM_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static uint8_t const s_cfg_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, NCM_TOTAL_LEN, 0, 100),
    TUD_CDC_NCM_DESCRIPTOR(ITF_NUM_NET, STRID_NET, STRID_MAC,
                           EPNUM_NET_NOTIF, 64,
                           EPNUM_NET_OUT, EPNUM_NET_IN,
                           CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};

static char s_mac_str[2 * 6 + 1] = {0};

static const char *s_str_desc[] = {
    (const char[]){0x09, 0x04},
    "ESPNetLink",
    "ESPNetLink USB Ethernet (NCM)",
    "000000000000",
    "",      // CDC interface string (not used by our descriptor, but keeps IDs aligned)
    "USB net",
    s_mac_str, // MAC string (12 hex chars), also updated at runtime
};

#if CONFIG_TINYUSB_NET_MODE_NCM

// --------------------------------------------------------------------------------------
// Windows 10/11 NCM driver binding: BOS + MS OS 2.0 descriptor set (Compatible ID: WINNCM)
// --------------------------------------------------------------------------------------

#define MS_OS_20_DESC_LEN 0xB2
#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static uint8_t const s_desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1),
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return s_desc_bos;
}

// MS OS 2.0 descriptor set: compatible ID WINNCM + DeviceInterfaceGUIDs
static uint8_t const s_desc_ms_os_20[] = {
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Configuration subset header: length, type, configuration index, reserved, configuration total length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    // Function subset header: length, type, first interface, reserved, subset length
    // NOTE: The first interface is assumed to be 0 for the NCM function in esp_tinyusb default descriptors.
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

    // Compatible ID: "WINNCM"
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'N', 'C', 'M', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Registry property descriptor (DeviceInterfaceGUIDs)
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050),

    // bPropertyData (UTF-16): {7B36B70A-12A1-4E73-9F0B-4B2B7C4B88C8}
    '{', 0x00, '7', 0x00, 'B', 0x00, '3', 0x00, '6', 0x00, 'B', 0x00, '7', 0x00, '0', 0x00, 'A', 0x00, '-', 0x00,
    '1', 0x00, '2', 0x00, 'A', 0x00, '1', 0x00, '-', 0x00, '4', 0x00, 'E', 0x00, '7', 0x00, '3', 0x00, '-', 0x00,
    '9', 0x00, 'F', 0x00, '0', 0x00, 'B', 0x00, '-', 0x00, '4', 0x00, 'B', 0x00, '2', 0x00, 'B', 0x00, '7', 0x00,
    'C', 0x00, '4', 0x00, 'B', 0x00, '8', 0x00, '8', 0x00, 'C', 0x00, '8', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(s_desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect MS OS 2.0 descriptor size");

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) {
        return false;
    }

    // Windows requests MS OS 2.0 descriptor set with bRequest=1, wIndex=7
    if (request->bRequest == 1 && request->wIndex == 7) {
        uint16_t total_len = 0;
        memcpy(&total_len, s_desc_ms_os_20 + 8, 2);
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)s_desc_ms_os_20, total_len);
    }

    return false;
}

// --------------------------------------------------------------------------------------
// tinyusb_net callbacks (frames are currently dropped)
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// lwIP + DHCP server for a working USB-NCM network (host gets an IP)
// --------------------------------------------------------------------------------------

static struct netif s_usbnetif;
static bool s_lwip_ready;

#define USBNET_IP4(a, b, c, d) { PP_HTONL(LWIP_MAKEU32((a), (b), (c), (d))) }

static const ip4_addr_t s_ipaddr = USBNET_IP4(192, 168, 7, 1);
static const ip4_addr_t s_netmask = USBNET_IP4(255, 255, 255, 0);
static const ip4_addr_t s_gateway = USBNET_IP4(0, 0, 0, 0);

static dhcp_entry_t s_dhcp_entries[] = {
    {{0}, USBNET_IP4(192, 168, 7, 2), 24 * 60 * 60},
    {{0}, USBNET_IP4(192, 168, 7, 3), 24 * 60 * 60},
    {{0}, USBNET_IP4(192, 168, 7, 4), 24 * 60 * 60},
    {{0}, USBNET_IP4(192, 168, 7, 5), 24 * 60 * 60},
    {{0}, USBNET_IP4(192, 168, 7, 6), 24 * 60 * 60},
};

static dhcp_config_t s_dhcp_cfg = {

    // netif is set at runtime once s_usbnetif is created.
    .netif = NULL,
    .router = USBNET_IP4(192, 168, 7, 1),
    .port = 67,
    // Default to a public DNS; can be overridden later by usb_dev_ethernet_enable_sharing().
    .dns = USBNET_IP4(8, 8, 8, 8),
    .domain = "usb",
    .num_entry = (int)(sizeof(s_dhcp_entries) / sizeof(s_dhcp_entries[0])),
    .entries = s_dhcp_entries,
};

static err_t usbnet_linkoutput(struct netif *netif, struct pbuf *p)
{
    (void)netif;
    if (!tud_ready()) {
        static uint32_t s_not_ready_drops;
        s_not_ready_drops++;
        if ((s_not_ready_drops == 1) || ((s_not_ready_drops % 50) == 0)) {
            ESP_LOGW(TAG, "TX drop: tud_ready()=false (drops=%u)", (unsigned)s_not_ready_drops);
        }
        return ERR_IF;
    }

    uint16_t len = (uint16_t)p->tot_len;

    // Optional: detect DHCP replies (UDP 67 -> 68) leaving the device.
    // Helps debug “DHCP timeout” cases on Windows.
    static uint32_t s_tx_frames;
    static uint32_t s_tx_dhcp;
    s_tx_frames++;
    if (p->tot_len >= (u16_t)(SIZEOF_ETH_HDR + 20 + 8)) {
        uint8_t hdr[SIZEOF_ETH_HDR + 20 + 8] = {0};
        pbuf_copy_partial(p, hdr, sizeof(hdr), 0);
        const struct eth_hdr *eth = (const struct eth_hdr *)hdr;
        if (lwip_ntohs(eth->type) == ETHTYPE_IP) {
            const uint8_t *ip = (const uint8_t *)hdr + SIZEOF_ETH_HDR;
            const uint8_t ihl = (uint8_t)((ip[0] & 0x0F) * 4);
            const uint8_t proto = ip[9];
            if (proto == 17 && ihl >= 20 && sizeof(hdr) >= (SIZEOF_ETH_HDR + ihl + 8)) {
                const uint8_t *udp = ip + ihl;
                const uint16_t src = (uint16_t)((udp[0] << 8) | udp[1]);
                const uint16_t dst = (uint16_t)((udp[2] << 8) | udp[3]);
                if (src == 67 && dst == 68) {
                    s_tx_dhcp++;
                    if ((s_tx_dhcp == 1) || ((s_tx_dhcp % 10) == 0)) {
                        ESP_LOGI(TAG, "TX DHCP reply sent (dhcp_tx=%u tx_frames=%u)",
                                 (unsigned)s_tx_dhcp, (unsigned)s_tx_frames);
                    }

                    if (s_tx_dhcp == 1) {
                        uint32_t src_ip = (uint32_t)((ip[12] << 24) | (ip[13] << 16) | (ip[14] << 8) | ip[15]);
                        uint32_t dst_ip = (uint32_t)((ip[16] << 24) | (ip[17] << 16) | (ip[18] << 8) | ip[19]);
                        uint16_t ip_chk = (uint16_t)((ip[10] << 8) | ip[11]);
                        uint16_t udp_chk = (uint16_t)((udp[6] << 8) | udp[7]);
                        const uint8_t *d = eth->dest.addr;
                        const uint8_t *s = eth->src.addr;
                        ESP_LOGI(TAG,
                                 "DHCP TX hdr: dmac=%02x:%02x:%02x:%02x:%02x:%02x smac=%02x:%02x:%02x:%02x:%02x:%02x",
                                 d[0], d[1], d[2], d[3], d[4], d[5], s[0], s[1], s[2], s[3], s[4], s[5]);
                        ESP_LOGI(TAG, "DHCP TX ip: src=%" PRIu32 ", dst=%" PRIu32 ", ipchk=0x%04x udpsrc=%u udpdst=%u udpchk=0x%04x",
                                 (uint32_t)src_ip, (uint32_t)dst_ip, (unsigned)ip_chk, (unsigned)src, (unsigned)dst, (unsigned)udp_chk);
                    }
                }
            }
        }
    }
    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) {
        return ERR_MEM;
    }
    pbuf_copy_partial(p, buf, len, 0);

    // Free the allocated buffer from the TinyUSB task once it consumes the packet.
    esp_err_t err = tinyusb_net_send_sync(buf, len, buf, pdMS_TO_TICKS(250));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX failed: tinyusb_net_send_sync err=0x%x len=%u", (unsigned)err, (unsigned)len);
        free(buf);
        return (err == ESP_ERR_TIMEOUT) ? ERR_TIMEOUT : ERR_IF;
    }
    return ERR_OK;
}

static err_t usbnet_netif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));
    netif->mtu = CFG_TUD_NET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
    netif->name[0] = 'u';
    netif->name[1] = 's';
    netif->linkoutput = usbnet_linkoutput;
    netif->output = etharp_output;
    return ERR_OK;
}

static void usbnet_lwip_init_in_tcpip(void *arg)
{
    uint8_t *mac = (uint8_t *)arg;
    memset(&s_usbnetif, 0, sizeof(s_usbnetif));

    s_usbnetif.hwaddr_len = 6;
    memcpy(s_usbnetif.hwaddr, mac, 6);

    // Add netif and bring it up
    struct netif *added = netif_add(&s_usbnetif, (ip4_addr_t *)&s_ipaddr, (ip4_addr_t *)&s_netmask, (ip4_addr_t *)&s_gateway,
                                   NULL, usbnet_netif_init, ethernet_input);
    if (added == NULL) {
        ESP_LOGE(TAG, "netif_add failed");
        s_lwip_ready = false;
        return;
    }
    netif_set_up(&s_usbnetif);
    netif_set_link_up(&s_usbnetif);

    // Pin the DHCP server to this netif so it always replies via USB.
    s_dhcp_cfg.netif = &s_usbnetif;

    // Start a tiny DHCP server so the host gets an IPv4 address.
    err_t dh_err = dhserv_init(&s_dhcp_cfg);
    if (dh_err != ERR_OK) {
        ESP_LOGE(TAG, "DHCP server start failed: dhserv_init err=%d", (int)dh_err);
    } else {
        ESP_LOGI(TAG, "DHCP server started on UDP/%u", (unsigned)s_dhcp_cfg.port);
    }

    s_lwip_ready = true;
}

typedef struct {
    uint32_t dns_ipv4_addr;
} usbnet_sharing_cfg_t;

static void usbnet_enable_sharing_in_tcpip(void *arg)
{
    usbnet_sharing_cfg_t *cfg = (usbnet_sharing_cfg_t *)arg;

    // Update DHCP DNS if provided
    if (cfg && cfg->dns_ipv4_addr != 0) {
        s_dhcp_cfg.dns.addr = cfg->dns_ipv4_addr;
    }

    // Restart DHCP server to ensure new clients pick up router/DNS.
    dhserv_free();
    err_t dh_err = dhserv_init(&s_dhcp_cfg);
    if (dh_err != ERR_OK) {
        ESP_LOGE(TAG, "DHCP server restart failed: dhserv_init err=%d", (int)dh_err);
    } else {
        ESP_LOGI(TAG, "DHCP server restarted (router=%" PRIu32 ", dns=%" PRIu32 ")",
                 (uint32_t)lwip_ntohl(s_dhcp_cfg.router.addr), (uint32_t)lwip_ntohl(s_dhcp_cfg.dns.addr));
    }

    // Enable NAT for the USB LAN (192.168.7.1). Requires CONFIG_LWIP_IP_FORWARD=y and CONFIG_LWIP_IPV4_NAPT=y.
#if CONFIG_LWIP_IPV4_NAPT
    ip_napt_enable(s_ipaddr.addr, 1);
#else
    ESP_LOGW(TAG, "NAT not enabled: set CONFIG_LWIP_IPV4_NAPT=y to share upstream internet");
#endif

    free(cfg);
}

esp_err_t usb_dev_ethernet_enable_sharing(uint32_t dns_ipv4_addr)
{
    if (!s_lwip_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    usbnet_sharing_cfg_t *cfg = (usbnet_sharing_cfg_t *)calloc(1, sizeof(*cfg));
    if (!cfg) {
        return ESP_ERR_NO_MEM;
    }
    cfg->dns_ipv4_addr = dns_ipv4_addr;

    err_t cb_err = tcpip_callback_with_block(usbnet_enable_sharing_in_tcpip, cfg, 1);
    if (cb_err != ERR_OK) {
        free(cfg);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "USB NCM sharing enabled (NAT + DHCP router/DNS)");
    return ESP_OK;
}

static void usbnet_link_up_task(void *arg)
{
    (void)arg;
    // Wait until host configured the device (TinyUSB mounted)
    while (!tud_ready()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Notify host that the link is up
    tud_network_link_state(0, true);
    vTaskDelete(NULL);
}

static esp_err_t usbnet_rx_callback(void *buffer, uint16_t len, void *ctx)
{
    (void)ctx;
    if (!s_lwip_ready) {
        return ESP_OK;
    }

    static uint32_t s_rx_frames;
    static uint32_t s_rx_arp;
    static uint32_t s_rx_ipv4;
    static uint32_t s_rx_dhcp;
    s_rx_frames++;

    if (len >= (uint16_t)SIZEOF_ETH_HDR) {
        const struct eth_hdr *eth = (const struct eth_hdr *)buffer;
        const uint16_t et = lwip_ntohs(eth->type);
        if (et == ETHTYPE_ARP) {
            s_rx_arp++;
        } else if (et == ETHTYPE_IP) {
            s_rx_ipv4++;
            // Very small DHCP detector (IPv4+UDP, ports 68->67)
            if (len >= (uint16_t)(SIZEOF_ETH_HDR + 20 + 8)) {
                const uint8_t *ip = (const uint8_t *)buffer + SIZEOF_ETH_HDR;
                const uint8_t ihl = (uint8_t)((ip[0] & 0x0F) * 4);
                const uint8_t proto = ip[9];
                if (proto == 17 && ihl >= 20 && len >= (uint16_t)(SIZEOF_ETH_HDR + ihl + 8)) {
                    const uint8_t *udp = ip + ihl;
                    const uint16_t src = (uint16_t)((udp[0] << 8) | udp[1]);
                    const uint16_t dst = (uint16_t)((udp[2] << 8) | udp[3]);
                    if (src == 68 && dst == 67) {
                        s_rx_dhcp++;
                        if ((s_rx_dhcp == 1) || ((s_rx_dhcp % 10) == 0)) {
                            ESP_LOGI(TAG, "RX DHCP discover/request seen (dhcp=%u frames=%u arp=%u ipv4=%u)",
                                     (unsigned)s_rx_dhcp, (unsigned)s_rx_frames, (unsigned)s_rx_arp, (unsigned)s_rx_ipv4);
                        }
                    }
                }
            }
        }

        if ((s_rx_frames == 1) || ((s_rx_frames % 200) == 0)) {
            ESP_LOGI(TAG, "RX stats: frames=%u arp=%u ipv4=%u dhcp=%u", (unsigned)s_rx_frames, (unsigned)s_rx_arp,
                     (unsigned)s_rx_ipv4, (unsigned)s_rx_dhcp);
        }
    }

    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (!p) {
        return ESP_ERR_NO_MEM;
    }
    pbuf_take(p, buffer, len);

    // TinyUSB's dhserver.c uses p->if_idx to look up the receiving netif.
    // When feeding frames from an external driver, this field may be left as 0 (unknown),
    // which can cause DHCP to respond via the wrong netif or not at all.
    // lwIP uses 1..N for if_idx; 0 means unspecified.
    p->if_idx = (uint8_t)(s_usbnetif.num + 1);

    // We receive a full Ethernet frame (L2). Feed it into lwIP's Ethernet input path
    // via the TCP/IP thread. Using tcpip_input() here is wrong because it expects an
    // IP packet (L3) without the Ethernet header, which breaks ARP and DHCP.
    err_t lwip_err = tcpip_inpkt(p, &s_usbnetif, ethernet_input);
    if (lwip_err != ERR_OK) {
        pbuf_free(p);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void usbnet_free_tx_buffer(void *buffer, void *ctx)
{
    (void)ctx;
    free(buffer);
}

esp_err_t usb_dev_ethernet_start(void)
{
    ESP_LOGI(TAG, "Starting USB Device Ethernet mode (CDC-NCM)");

    // Provide a stable MAC for the USB network interface.
    uint8_t mac[6] = {0};
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_STA), TAG, "esp_read_mac failed");
    // Locally administered + unicast
    mac[0] |= 0x02;
    mac[0] &= 0xFE;

    // Pre-fill the MAC string descriptor so iMACAddress always resolves.
    // (Windows NCM is picky; an invalid/missing MAC string can cause CM_PROB_FAILED_START)
    snprintf(s_mac_str, sizeof(s_mac_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Ensure lwIP/tcpip thread is running (required for tcpip_input and DHCP server).
    // This is safe even if other parts of the app never use esp_netif.
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");

    // Install TinyUSB device stack with runtime-supplied descriptors.
    // (Matches Espressif examples: driver install first, then tinyusb_net_init)
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = &s_dev_desc;
    tusb_cfg.descriptor.full_speed_config = s_cfg_desc;
    tusb_cfg.descriptor.string = s_str_desc;
    tusb_cfg.descriptor.string_count = (int)(sizeof(s_str_desc) / sizeof(s_str_desc[0]));
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb_driver_install failed");

    tinyusb_net_config_t net_cfg = {
        .mac_addr = {mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]},
        .on_recv_callback = usbnet_rx_callback,
        .free_tx_buffer = usbnet_free_tx_buffer,
        .on_init_callback = NULL,
        .user_context = NULL,
    };
    ESP_RETURN_ON_ERROR(tinyusb_net_init(&net_cfg), TAG, "tinyusb_net_init failed");

    // Create lwIP netif + DHCP server in TCP/IP thread context.
    s_lwip_ready = false;
    err_t cb_err = tcpip_callback_with_block(usbnet_lwip_init_in_tcpip, mac, 1);
    ESP_RETURN_ON_FALSE(cb_err == ERR_OK, ESP_FAIL, TAG, "tcpip_callback_with_block failed (%d)", (int)cb_err);

    // NCM drivers (especially on Windows) may require an explicit link-up notification.
    (void)xTaskCreate(usbnet_link_up_task, "usbnet_link", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "USB NCM IPv4: 192.168.7.1/24 (DHCP server enabled)");

#if CONFIG_LWIP_IP_FORWARD
#if CONFIG_LWIP_IPV4_NAPT
    ESP_LOGI(TAG, "USB NCM is up. Sharing is supported; enable it after upstream is connected (usb_dev_ethernet_enable_sharing)");
#else
    ESP_LOGW(TAG, "USB NCM is up. NAT disabled (CONFIG_LWIP_IPV4_NAPT=n), so internet sharing will not work.");
#endif
#else
    ESP_LOGW(TAG, "USB NCM is up. IP forwarding disabled (CONFIG_LWIP_IP_FORWARD=n), so internet sharing will not work.");
#endif
    return ESP_OK;
}

#else

esp_err_t usb_dev_ethernet_start(void)
{
    ESP_LOGE(TAG, "USB Ethernet mode requires CONFIG_TINYUSB_NET_MODE_NCM=y (menuconfig -> TinyUSB Stack -> Network mode -> NCM)");
    return ESP_ERR_NOT_SUPPORTED;
}

#endif // CONFIG_TINYUSB_NET_MODE_NCM
