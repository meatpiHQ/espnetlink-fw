/**
 * @file imu_manager_events.c
 * @brief event_manager glue: `imu.motion {state}` (active/stationary)
 *        and `imu.bump {axes}` from the subscriber queue.
 */
#include <stdio.h>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "event_manager.h"

#include "imu_manager.h"

static QueueHandle_t s_q;
static StaticQueue_t s_q_buf;              /* internal: FreeRTOS object */
static uint8_t s_q_store[8 * sizeof(imu_manager_event_t)]
    EXT_RAM_BSS_ATTR;
static TaskHandle_t s_task;
static StaticTask_t s_tcb;                 /* internal: FreeRTOS object */
static StackType_t s_stack[2560] EXT_RAM_BSS_ATTR;

static void drain_task(void *arg)
{
    (void)arg;

    while (true)
    {
        imu_manager_event_t ie;

        if (xQueueReceive(s_q, &ie, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        em_event_t ev = { 0 };

        snprintf(ev.source, sizeof(ev.source), "imu");

        switch (ie.type)
        {
            case IMU_MANAGER_EVENT_WOM:
                snprintf(ev.name, sizeof(ev.name), "bump");
                ev.kv[0] = em_kv_i64("axes", ie.wom_axes);
                ev.n = 1;
                break;
            case IMU_MANAGER_EVENT_ACTIVE:
            case IMU_MANAGER_EVENT_STATIONARY:
                snprintf(ev.name, sizeof(ev.name), "motion");
                ev.kv[0] = em_kv_str(
                    "state", (ie.type == IMU_MANAGER_EVENT_ACTIVE)
                                 ? "active" : "stationary");
                ev.n = 1;
                break;
            default:
                continue;   /* raw SMD pulses: activity covers it       */
        }

        (void)event_manager_publish(&ev);
    }
}

void imu_events_register(void)
{
    static const em_key_decl_t MOTION_KEYS[] =
    {
        { "state", EM_VAL_STR },
    };
    static const em_source_decl_t MOTION =
    {
        .source = "imu", .name = "motion",
        .description = "sustained-motion activity changed "
                       "(state: active|stationary)",
        .keys = MOTION_KEYS, .n_keys = 1,
    };
    static const em_key_decl_t BUMP_KEYS[] =
    {
        { "axes", EM_VAL_I64 },
    };
    static const em_source_decl_t BUMP =
    {
        .source = "imu", .name = "bump",
        .description = "wake-on-motion pulse (door slam, knock; "
                       "axes bit0=Z bit1=Y bit2=X)",
        .keys = BUMP_KEYS, .n_keys = 1,
    };

    (void)event_manager_declare_source(&MOTION);
    (void)event_manager_declare_source(&BUMP);
}

void imu_events_start(void)
{
    if (s_task != NULL)
    {
        return;
    }

    s_q = xQueueCreateStatic(8, sizeof(imu_manager_event_t), s_q_store,
                             &s_q_buf);

    if (s_q == NULL || imu_manager_subscribe(s_q) != ESP_OK)
    {
        return;
    }

    s_task = xTaskCreateStatic(drain_task, "imu_events",
                               sizeof(s_stack) / sizeof(s_stack[0]),
                               NULL, 3, s_stack, &s_tcb);
}
