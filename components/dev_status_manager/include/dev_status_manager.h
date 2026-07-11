/**
 * @file dev_status_manager.h
 * @brief WiCAN device status manager (core component).
 *
 * Owns the ONE device-status event group. Feature components publish their
 * state by setting/clearing well-known bits; consumers (LED logic, sleep
 * manager, status API, UI) read snapshots or block on bit combinations.
 * Nobody keeps a private "is X connected" flag — this is the single source
 * of truth for live device state (Architecture §2: own the resource, let
 * others register into it).
 *
 * Bits are ephemeral runtime state — they reset on reboot and are NOT
 * settings. Only the lower 24 bits exist (FreeRTOS reserves the top 8).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Well-known device status bits. Add new ones here (≤ BIT23) with a matching
 * name entry in dev_status_manager_fmt.c. */
#define DEV_STATUS_BIT_AWAKE            BIT0
#define DEV_STATUS_BIT_SLEEP            BIT1
#define DEV_STATUS_BIT_STA_CONNECTED    BIT2
#define DEV_STATUS_BIT_MQTT_CONNECTED   BIT3
#define DEV_STATUS_BIT_BLE_CONNECTED    BIT4
#define DEV_STATUS_BIT_SDCARD_MOUNTED   BIT5
#define DEV_STATUS_BIT_BLE_ENABLED      BIT6
#define DEV_STATUS_BIT_STA_ENABLED      BIT7
#define DEV_STATUS_BIT_AP_ENABLED       BIT8
#define DEV_STATUS_BIT_AUTOPID_ENABLED  BIT9
#define DEV_STATUS_BIT_HOME_MODE        BIT10
#define DEV_STATUS_BIT_DRIVE_MODE       BIT11
#define DEV_STATUS_BIT_SMARTCONNECT     BIT12
#define DEV_STATUS_BIT_STA_AP_OVERLAP   BIT13
#define DEV_STATUS_BIT_TIME_SYNCED      BIT14
#define DEV_STATUS_BIT_VPN_ENABLED      BIT15
#define DEV_STATUS_BIT_WAKE_VOLTAGE_OK  BIT16
#define DEV_STATUS_BIT_ETH_CONNECTED    BIT17
#define DEV_STATUS_BIT_AUTOPID_IDLE     BIT18
#define DEV_STATUS_BIT_MOTION           BIT19 /* imu_manager activity */
/* interface_manager's runtime arbitration state (WHY an interface is
 * down): the configured surface is untouched, the policy holds it */
#define DEV_STATUS_BIT_STA_SUSPENDED    BIT20
#define DEV_STATUS_BIT_AP_SUSPENDED     BIT21
#define DEV_STATUS_BIT_BLE_SUSPENDED    BIT22

/** Any upstream network path (WiFi STA or wired ETH). */
#define DEV_STATUS_NETWORK_CONNECTED_MASK \
    (DEV_STATUS_BIT_STA_CONNECTED | DEV_STATUS_BIT_ETH_CONNECTED)

#define DEV_STATUS_ALL_BITS ((EventBits_t)0x00FFFFFFu)

/** Create the event group and capture running-app/partition info. */
esp_err_t dev_status_manager_init(void);

/** Lifecycle uniformity (§3); passive component — both trivial. */
esp_err_t dev_status_manager_start(void);
esp_err_t dev_status_manager_stop(void);

/* ---- publish ------------------------------------------------------------- */

void dev_status_manager_set(EventBits_t bits);
void dev_status_manager_clear(EventBits_t bits);
void dev_status_manager_clear_all(void);

/* ---- read / wait ----------------------------------------------------------- */

EventBits_t dev_status_manager_get(void);
bool dev_status_manager_is_set(EventBits_t bit);
bool dev_status_manager_all_set(EventBits_t bits);
bool dev_status_manager_any_set(EventBits_t bits);

/** Block until ALL of @p bits are set (bits are not cleared on exit). */
EventBits_t dev_status_manager_wait_all(EventBits_t bits, TickType_t timeout);

/** Block until ANY of @p bits is set (bits are not cleared on exit). */
EventBits_t dev_status_manager_wait_any(EventBits_t bits, TickType_t timeout);

/* ---- identity / uptime helpers (for status transports) --------------------- */

/** Name of a single status bit ("sta_connected", ...); "unknown" if unmapped. */
const char *dev_status_manager_bit_name(EventBits_t bit);

/** Uptime as "HH:MM:SS" or "<d>d HH:MM:SS". Returns chars written, 0 on error. */
size_t dev_status_manager_format_uptime(char *buf, size_t buf_len);

/** Running app version string (from the app descriptor); "" before init. */
const char *dev_status_manager_app_version(void);

/** Running partition label ("ota_0"/"ota_1"/...); "" before init. */
const char *dev_status_manager_partition_label(void);

/**
 * The device id: 12 lowercase hex chars of the SoftAP MAC (legacy
 * hw_config_get_device_id). THE one identity every derived name builds on —
 * BLE device name ("WiCAN_<id>"), STA hostname ("wican_<id>"), AP SSID.
 * Cached; safe to call from any component after boot.
 */
const char *dev_status_manager_device_id(void);

/* ---- memory observability (Architecture §12b) ------------------------------ */

/** One heap region's health. `largest_block` is THE fragmentation signal:
 *  a large gap between `free` and `largest_block` means the free space is
 *  shattered and big allocations will fail despite "enough" free bytes. */
typedef struct
{
    uint32_t total;         /* capability heap size at boot                */
    uint32_t free;          /* free bytes right now                        */
    uint32_t min_free;      /* lowest free ever (high-water mark)          */
    uint32_t largest_block; /* biggest single allocation possible now      */
} dev_status_heap_t;

typedef struct
{
    dev_status_heap_t internal; /* MALLOC_CAP_INTERNAL — the scarce one    */
    dev_status_heap_t psram;    /* MALLOC_CAP_SPIRAM                       */
} dev_status_memory_t;

/** Register the `version`/`status` CLI commands with cmdline_manager.
 *  Called INTERNALLY on the settings boot apply when the `cli` setting is
 *  true (default) — main no longer wires it. */
esp_err_t dev_status_manager_register_cli(void);

/** Register the settings descriptor ({cli}). Init runs before
 *  settings_manager_init, so the composition root calls this separately
 *  (the log_manager_register_settings pattern). */
esp_err_t dev_status_manager_register_settings(void);

/** Snapshot both heaps. Cheap; poll-friendly. */
esp_err_t dev_status_manager_memory(dev_status_memory_t *out);

/* ---- task monitoring (2026-07-08) ------------------------------------------ */

/** One task's live stats. `stack_hw` is stack that was NEVER used (bytes;
 *  small = close to overflow). `runtime_us` is the cumulative esp_timer
 *  time the scheduler charged to the task — CPU% comes from DELTAS
 *  between two snapshots: task_delta / (total_delta * cores). */
typedef struct
{
    char     name[16];
    char     state;      /* X running, R ready, B blocked, S suspended,
                            D deleted-pending-cleanup                    */
    int8_t   core;       /* 0 / 1, -1 = unpinned                        */
    uint8_t  prio;
    uint32_t stack_hw;   /* stack high-water: unused bytes remaining    */
    uint64_t runtime_us; /* cumulative run time (u64 — never wraps)     */
} dev_status_task_t;

/**
 * Snapshot every task into @p out (capacity @p cap): fills @p out_count
 * and @p out_total_us (scheduler time since boot, PER CORE — multiply by
 * core count for the CPU% denominator; both nullable). Sorted by
 * runtime, busiest first. Needs CONFIG_FREERTOS_USE_TRACE_FACILITY (+
 * _GENERATE_RUN_TIME_STATS for runtimes); ESP_ERR_NOT_SUPPORTED without.
 * ESP_ERR_NO_MEM on scratch-alloc failure; if @p cap is too small the
 * BUSIEST @p cap tasks are kept (out_count says how many were written).
 */
esp_err_t dev_status_manager_task_stats(dev_status_task_t *out, size_t cap,
                                        size_t *out_count,
                                        uint64_t *out_total_us);

/** Die temperature in °C (ESP32-S3 internal sensor; installed lazily on
 *  first call, -10..80 °C measurement range). */
esp_err_t dev_status_manager_temperature(float *out_c);

#ifdef __cplusplus
}
#endif

/* ---- change notification (event_manager glue and friends) ------------------ */

/** Called after any bit change: @p changed = XOR of old/new, @p now =
 *  the bits after. Runs in the SETTER's context — keep it non-blocking
 *  (queue sends only). ≤2 subscribers, registered for life. */
typedef void (*dev_status_change_cb_t)(EventBits_t changed,
                                       EventBits_t now);
esp_err_t dev_status_manager_subscribe_changes(dev_status_change_cb_t cb);

/** event_manager glue: declare the `status.bit` source + hook changes
 *  (dev_status_manager_events.c). Called from init. */
void dsm_events_register(void);
