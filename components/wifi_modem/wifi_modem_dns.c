/**
 * @file wifi_modem_dns.c
 * @brief Minimal DNS proxy for the WiFi AP clients (pattern lifted from
 *        usb_dev_ethernet's usbnet_dns_proxy). One query in flight at a
 *        time — plenty for a WiCAN + a phone; resolvers answer in ~100 ms
 *        over LTE.
 *
 * Binds <AP address>:53 with SO_REUSEADDR — never INADDR_ANY, so the USB
 * link's 192.168.7.1:53 stays whatever usb_dev_ethernet decides (closed in
 * control-channel mode, its own proxy when sharing) and lwIP delivers each
 * query to the pcb bound to the matching address. A busy port (retrying
 * quietly) is left for the legacy single-proxy case.
 */
#include "wifi_modem_private.h"

#include <stdint.h>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

static const char *TAG = "wifi_modem";

#define DNS_PORT        53
#define DNS_TASK_STACK  4096
#define BIND_RETRY_MS   30000
#define RECV_POLL_MS    1000

static TaskHandle_t s_task;
static volatile bool s_run;
static volatile uint32_t s_upstream;       /* network order, 0 = unknown */
static uint32_t s_local;                   /* bind address, network order */

/* §2: static stack in PSRAM (UDP forwarder — no flash writes) */
static StaticTask_t s_tcb;                               /* internal: TCB */
static StackType_t s_stack[DNS_TASK_STACK] EXT_RAM_BSS_ATTR;
static uint8_t s_buf[512] EXT_RAM_BSS_ATTR;

void wifi_modem_dns_set_upstream(uint32_t addr_be)
{
    s_upstream = addr_be;
}

static int open_listener(void)
{
    int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sock < 0)
    {
        return -1;
    }

    int reuse = 1;

    (void)lwip_setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr = { 0 };

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = lwip_htons(DNS_PORT);
    bind_addr.sin_addr.s_addr = s_local;

    if (lwip_bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0)
    {
        (void)lwip_close(sock);
        return -1;
    }

    /* bounded recv so the loop can notice a stop request */
    struct timeval tv = { .tv_sec = RECV_POLL_MS / 1000, .tv_usec = 0 };

    (void)lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return sock;
}

static int open_upstream(void)
{
    int up = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (up < 0)
    {
        return -1;
    }

    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };

    (void)lwip_setsockopt(up, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return up;
}

/* Relay one query; false = the upstream socket should be recycled. */
static bool relay(int sock, int up, int n, uint32_t target,
                  const struct sockaddr_in *client, socklen_t clen)
{
    struct sockaddr_in upstream = { 0 };

    upstream.sin_family = AF_INET;
    upstream.sin_port = lwip_htons(DNS_PORT);
    upstream.sin_addr.s_addr = target;

    if (lwip_sendto(up, s_buf, n, 0, (struct sockaddr *)&upstream,
                    sizeof(upstream)) < 0)
    {
        return false;
    }

    struct sockaddr_in from = { 0 };
    socklen_t flen = sizeof(from);
    int rn = lwip_recvfrom(up, s_buf, sizeof(s_buf), 0,
                           (struct sockaddr *)&from, &flen);

    if (rn <= 0)
    {
        return false;
    }

    (void)lwip_sendto(sock, s_buf, rn, 0, (const struct sockaddr *)client, clen);
    return true;
}

static void dns_task(void *arg)
{
    (void)arg;

    int sock = -1;
    int up = -1;
    uint32_t up_addr = 0;
    bool busy_logged = false;

    while (s_run)
    {
        if (sock < 0)
        {
            sock = open_listener();
            if (sock < 0)
            {
                /* §10: the state once at WARN, the retries at DEBUG */
                if (!busy_logged)
                {
                    ESP_LOGW(TAG, "DNS proxy: UDP/%d busy (usbnet proxy owns "
                                  "it?) - idling", DNS_PORT);
                    busy_logged = true;
                }
                else
                {
                    ESP_LOGD(TAG, "DNS proxy: UDP/%d still busy", DNS_PORT);
                }
                for (int i = 0; i < BIND_RETRY_MS / RECV_POLL_MS && s_run; i++)
                {
                    vTaskDelay(pdMS_TO_TICKS(RECV_POLL_MS));
                }
                continue;
            }
            ESP_LOGI(TAG, "DNS proxy listening on the AP address, UDP/%d",
                     DNS_PORT);
        }

        struct sockaddr_in client = { 0 };
        socklen_t clen = sizeof(client);
        int n = lwip_recvfrom(sock, s_buf, sizeof(s_buf), 0,
                              (struct sockaddr *)&client, &clen);

        if (n <= 0)
        {
            continue; /* timeout (stop poll) or a transient error */
        }

        uint32_t target = wmc_pick_upstream_dns(s_upstream);

        if (up >= 0 && up_addr != target)
        {
            (void)lwip_close(up);
            up = -1;
        }
        if (up < 0)
        {
            up = open_upstream();
            if (up < 0)
            {
                continue;
            }
            up_addr = target;
        }

        if (!relay(sock, up, n, target, &client, clen))
        {
            (void)lwip_close(up);
            up = -1;
        }
    }

    if (up >= 0)
    {
        (void)lwip_close(up);
    }
    if (sock >= 0)
    {
        (void)lwip_close(sock);
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t wifi_modem_dns_start(uint32_t local_be)
{
    if (s_task != NULL)
    {
        return ESP_OK;
    }

    s_local = local_be;
    s_run = true;
    s_task = xTaskCreateStaticPinnedToCore(dns_task, "wm_dns", DNS_TASK_STACK,
                                           NULL, 9, s_stack, &s_tcb, 0);
    if (s_task == NULL)
    {
        s_run = false;
        ESP_LOGE(TAG, "DNS proxy task not created");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void wifi_modem_dns_stop(void)
{
    s_run = false;

    /* the task notices within one RECV_POLL_MS and exits */
    for (int i = 0; i < 30 && s_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
