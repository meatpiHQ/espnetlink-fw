#include "private/usbnet_dns_proxy.h"

#include <stdint.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"

#include "private/usbnet_state.h"

static const char *TAG = "usb_dev_eth";

#define USBNET_DNS_PORT 53

static TaskHandle_t s_dns_task = NULL;
static int s_dns_sock = -1;
/* Persistent upstream UDP socket – reused across queries to avoid per-query
 * socket creation/teardown overhead, which adds noticeable latency on LTE. */
static int s_up_sock = -1;
static uint32_t s_up_sock_dns_addr = 0; /* tracks which upstream s_up_sock targets */

static bool usbnet_is_usb_client_ipv4(uint32_t be_addr)
{
    return ((lwip_ntohl(be_addr) & 0xFFFFFF00u) == 0xC0A80700u);
}

static void usbnet_dns_proxy_task(void *arg)
{
    (void)arg;

    uint8_t buf[512];

    while (true)
    {
        usbnet_state_t *st = usbnet_state();

        if (!st->lwip_ready)
        {
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (s_dns_sock < 0)
        {
            int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock < 0)
            {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            struct sockaddr_in bind_addr = { 0 };
            bind_addr.sin_family = AF_INET;
            bind_addr.sin_port = lwip_htons(USBNET_DNS_PORT);
            bind_addr.sin_addr.s_addr = lwip_htonl(INADDR_ANY);

            if (lwip_bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0)
            {
                (void)lwip_close(sock);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            s_dns_sock = sock;
            ESP_LOGI(TAG, "DNS proxy listening on UDP/%u", (unsigned)USBNET_DNS_PORT);
        }

        struct sockaddr_in client = { 0 };
        socklen_t clen = sizeof(client);
        int n = lwip_recvfrom(s_dns_sock, buf, sizeof(buf), 0, (struct sockaddr *)&client, &clen);
        if (n <= 0)
        {
            continue;
        }

        if (!usbnet_is_usb_client_ipv4(client.sin_addr.s_addr))
        {
            continue;
        }

        ip4_addr_t upstream = st->upstream_dns;
        if (ip4_addr_isany_val(upstream))
        {
            IP4_ADDR(&upstream, 8, 8, 8, 8);
        }

        /* (Re-)create the upstream socket if it doesn't exist yet or the
         * upstream DNS server has changed (e.g. after PPP reconnect). */
        if (s_up_sock >= 0 && s_up_sock_dns_addr != upstream.addr)
        {
            lwip_close(s_up_sock);
            s_up_sock = -1;
        }

        if (s_up_sock < 0)
        {
            s_up_sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (s_up_sock < 0)
            {
                continue;
            }
            struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
            (void)lwip_setsockopt(s_up_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            s_up_sock_dns_addr = upstream.addr;
        }

        struct sockaddr_in upstream_addr = { 0 };
        upstream_addr.sin_family = AF_INET;
        upstream_addr.sin_port = lwip_htons(53);
        upstream_addr.sin_addr.s_addr = upstream.addr;

        if (lwip_sendto(s_up_sock, buf, n, 0, (struct sockaddr *)&upstream_addr, sizeof(upstream_addr)) < 0)
        {
            lwip_close(s_up_sock);
            s_up_sock = -1;
            continue;
        }

        struct sockaddr_in from = { 0 };
        socklen_t flen = sizeof(from);
        int rn = lwip_recvfrom(s_up_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &flen);
        if (rn <= 0)
        {
            /* Timeout or error – close and recreate on the next query. */
            lwip_close(s_up_sock);
            s_up_sock = -1;
            continue;
        }

        (void)lwip_sendto(s_dns_sock, buf, rn, 0, (struct sockaddr *)&client, clen);
    }
}

void usbnet_dns_proxy_start(void)
{
    if (s_dns_task)
    {
        return;
    }

    xTaskCreatePinnedToCore(usbnet_dns_proxy_task, "usbnet_dns", 4096, NULL, 9, &s_dns_task, 0);
}
