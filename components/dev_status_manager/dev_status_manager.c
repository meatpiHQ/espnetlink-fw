/**
 * @file dev_status_manager.c
 * @brief The one device-status event group: publish/read/wait + identity
 *        helpers. Bit changes log at DEBUG (they can be per-connection
 *        chatty), lifecycle at INFO (Coding Standard §10).
 */
#include "dev_status_manager.h"

#include <stdlib.h>
#include <string.h>

#include <stdio.h>

#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "driver/temperature_sensor.h"
#include "freertos/task.h"

#include "log_manager.h"

#include "dev_status_manager_private.h"

static const char *TAG = "dev_status_manager";

static EventGroupHandle_t s_events;
static StaticEventGroup_t s_events_buf; /* internal: FreeRTOS object */

static const esp_partition_t *s_running_partition;
static esp_app_desc_t s_app_desc;
static bool s_inited;

static void log_bits(const char *action, EventBits_t bits)
{
    for (int i = 0; i < 24; i++)
    {
        if ((bits & (1u << i)) != 0)
        {
            ESP_LOGD(TAG, "%s %s", action, dsm_bit_name(1u << i));
        }
    }
}

esp_err_t dev_status_manager_init(void)
{
    if (s_inited)
    {
        return ESP_OK;
    }

    static const log_descriptor_t LOG_DESC =
        { "dev_status_manager", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC); /* per-TAG level control (§9.2) */

    s_events = xEventGroupCreateStatic(&s_events_buf);

    if (s_events == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_running_partition = esp_ota_get_running_partition();

    if (s_running_partition == NULL ||
        esp_ota_get_partition_description(s_running_partition, &s_app_desc)
            != ESP_OK)
    {
        memset(&s_app_desc, 0, sizeof(s_app_desc));
        ESP_LOGW(TAG, "running app descriptor unavailable");
    }

    s_inited = true;
    ESP_LOGI(TAG, "up (partition=%s, version=%s)",
             dev_status_manager_partition_label(),
             dev_status_manager_app_version());
    dsm_events_register();
    return ESP_OK;
}

esp_err_t dev_status_manager_start(void)
{
    return s_inited ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t dev_status_manager_stop(void)
{
    return ESP_OK; /* passive: nothing to stop */
}

/* change subscribers (event_manager glue): setter-context, no blocking */
static dev_status_change_cb_t s_change_cbs[2];

esp_err_t dev_status_manager_subscribe_changes(dev_status_change_cb_t cb)
{
    for (size_t i = 0;
         i < sizeof(s_change_cbs) / sizeof(s_change_cbs[0]); i++)
    {
        if (s_change_cbs[i] == NULL)
        {
            s_change_cbs[i] = cb;
            return ESP_OK;
        }
    }

    return ESP_ERR_NO_MEM;
}

static void notify_changes(EventBits_t before, EventBits_t now)
{
    EventBits_t changed = (before ^ now) & DEV_STATUS_ALL_BITS;

    if (changed == 0)
    {
        return;
    }

    for (size_t i = 0;
         i < sizeof(s_change_cbs) / sizeof(s_change_cbs[0]); i++)
    {
        if (s_change_cbs[i] != NULL)
        {
            s_change_cbs[i](changed, now);
        }
    }
}

void dev_status_manager_set(EventBits_t bits)
{
    if (s_events != NULL)
    {
        EventBits_t before = xEventGroupGetBits(s_events);

        xEventGroupSetBits(s_events, bits & DEV_STATUS_ALL_BITS);
        log_bits("set", bits);
        notify_changes(before, xEventGroupGetBits(s_events));
    }
}

void dev_status_manager_clear(EventBits_t bits)
{
    if (s_events != NULL)
    {
        EventBits_t before = xEventGroupGetBits(s_events);

        xEventGroupClearBits(s_events, bits & DEV_STATUS_ALL_BITS);
        log_bits("clear", bits);
        notify_changes(before, xEventGroupGetBits(s_events));
    }
}

void dev_status_manager_clear_all(void)
{
    if (s_events != NULL)
    {
        xEventGroupClearBits(s_events, DEV_STATUS_ALL_BITS);
        ESP_LOGD(TAG, "clear all");
    }
}

EventBits_t dev_status_manager_get(void)
{
    return (s_events != NULL) ? xEventGroupGetBits(s_events) : 0;
}

bool dev_status_manager_is_set(EventBits_t bit)
{
    return (dev_status_manager_get() & bit) != 0;
}

bool dev_status_manager_all_set(EventBits_t bits)
{
    return (dev_status_manager_get() & bits) == bits;
}

bool dev_status_manager_any_set(EventBits_t bits)
{
    return (dev_status_manager_get() & bits) != 0;
}

EventBits_t dev_status_manager_wait_all(EventBits_t bits, TickType_t timeout)
{
    if (s_events == NULL)
    {
        return 0;
    }

    return xEventGroupWaitBits(s_events, bits, pdFALSE, pdTRUE, timeout);
}

EventBits_t dev_status_manager_wait_any(EventBits_t bits, TickType_t timeout)
{
    if (s_events == NULL)
    {
        return 0;
    }

    return xEventGroupWaitBits(s_events, bits, pdFALSE, pdFALSE, timeout);
}

const char *dev_status_manager_bit_name(EventBits_t bit)
{
    return dsm_bit_name((uint32_t)bit);
}

size_t dev_status_manager_format_uptime(char *buf, size_t buf_len)
{
    return dsm_format_uptime((uint64_t)esp_timer_get_time(), buf, buf_len);
}

const char *dev_status_manager_app_version(void)
{
    return s_app_desc.version;
}

const char *dev_status_manager_partition_label(void)
{
    return (s_running_partition != NULL) ? s_running_partition->label : "";
}

static void fill_heap(uint32_t caps, dev_status_heap_t *h)
{
    h->total = (uint32_t)heap_caps_get_total_size(caps);
    h->free = (uint32_t)heap_caps_get_free_size(caps);
    h->min_free = (uint32_t)heap_caps_get_minimum_free_size(caps);
    h->largest_block = (uint32_t)heap_caps_get_largest_free_block(caps);
}

esp_err_t dev_status_manager_memory(dev_status_memory_t *out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    fill_heap(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT, &out->internal);
    fill_heap(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, &out->psram);
    return ESP_OK;
}

#if configUSE_TRACE_FACILITY == 1
static char task_state_char(eTaskState st)
{
    switch (st)
    {
        case eRunning:   return 'X';
        case eReady:     return 'R';
        case eBlocked:   return 'B';
        case eSuspended: return 'S';
        case eDeleted:   return 'D';
        default:         return '?';
    }
}

static int task_cmp_runtime_desc(const void *a, const void *b)
{
    const dev_status_task_t *ta = a;
    const dev_status_task_t *tb = b;

    return (tb->runtime_us > ta->runtime_us) -
           (tb->runtime_us < ta->runtime_us);
}
#endif

esp_err_t dev_status_manager_task_stats(dev_status_task_t *out, size_t cap,
                                        size_t *out_count,
                                        uint64_t *out_total_us)
{
#if configUSE_TRACE_FACILITY == 1
    if (out == NULL || cap == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* +4: tasks can spawn between the count and the snapshot */
    UBaseType_t slots = uxTaskGetNumberOfTasks() + 4;
    /* transient scratch; PSRAM heap keeps internal RAM untouched */
    TaskStatus_t *tasks = heap_caps_malloc(slots * sizeof(TaskStatus_t),
                                           MALLOC_CAP_SPIRAM |
                                               MALLOC_CAP_8BIT);

    if (tasks == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    configRUN_TIME_COUNTER_TYPE total = 0;
    UBaseType_t count = uxTaskGetSystemState(tasks, slots, &total);
    /* sort in a full-size scratch so a too-small @p cap keeps the
       BUSIEST tasks, not an arbitrary prefix */
    dev_status_task_t *all = heap_caps_calloc(count, sizeof(*all),
                                              MALLOC_CAP_SPIRAM |
                                                  MALLOC_CAP_8BIT);

    if (all == NULL)
    {
        heap_caps_free(tasks);
        return ESP_ERR_NO_MEM;
    }

    for (UBaseType_t i = 0; i < count; i++)
    {
        dev_status_task_t *t = &all[i];

        strlcpy(t->name, tasks[i].pcTaskName, sizeof(t->name));
        t->state = task_state_char(tasks[i].eCurrentState);
        t->prio = (uint8_t)tasks[i].uxCurrentPriority;
        t->stack_hw = (uint32_t)tasks[i].usStackHighWaterMark *
                      sizeof(StackType_t);
#if configGENERATE_RUN_TIME_STATS == 1
        t->runtime_us = (uint64_t)tasks[i].ulRunTimeCounter;
#endif
#if configTASKLIST_INCLUDE_COREID == 1
        t->core = (tasks[i].xCoreID == tskNO_AFFINITY)
                      ? -1
                      : (int8_t)tasks[i].xCoreID;
#else
        t->core = -1;
#endif
    }

    qsort(all, count, sizeof(*all), task_cmp_runtime_desc);

    size_t n = (count < cap) ? count : cap;

    memcpy(out, all, n * sizeof(*out));
    heap_caps_free(all);
    heap_caps_free(tasks);

    if (out_count != NULL)
    {
        *out_count = n;
    }

    if (out_total_us != NULL)
    {
        *out_total_us = (uint64_t)total;
    }

    return ESP_OK;
#else
    (void)out;
    (void)cap;
    (void)out_count;
    (void)out_total_us;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t dev_status_manager_temperature(float *out_c)
{
    if (out_c == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    static temperature_sensor_handle_t s_tsens; /* lazy, kept forever */

    if (s_tsens == NULL)
    {
        temperature_sensor_config_t cfg =
            TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        esp_err_t err = temperature_sensor_install(&cfg, &s_tsens);

        if (err != ESP_OK)
        {
            return err;
        }

        err = temperature_sensor_enable(s_tsens);

        if (err != ESP_OK)
        {
            temperature_sensor_uninstall(s_tsens);
            s_tsens = NULL;
            return err;
        }
    }

    return temperature_sensor_get_celsius(s_tsens, out_c);
}

const char *dev_status_manager_device_id(void)
{
    /* legacy hw_config_get_device_id formula: 12 lowercase hex chars of the
       SoftAP MAC — the one device identity every derived name builds on
       (BLE name "WiCAN_<id>", STA hostname "wican_<id>", AP SSID) */
    static char s_id[13];

    if (s_id[0] == '\0')
    {
        uint8_t mac[6] = { 0 };

        esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
        snprintf(s_id, sizeof(s_id), "%02x%02x%02x%02x%02x%02x", mac[0],
                 mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    return s_id;
}
