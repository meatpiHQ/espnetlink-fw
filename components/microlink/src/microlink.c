/**
 * @file microlink.c
 * @brief MicroLink v2 - Public API and Task Orchestration
 *
 * Creates all FreeRTOS tasks, queues, and event groups.
 * Provides the public API that the application calls.
 */

#include "microlink_internal.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include "lwip/sockets.h"

#ifdef CONFIG_ML_ENABLE_CELLULAR
#include "ml_cellular.h"
#endif

static const char *TAG = "microlink";

/* NVS keys */
#define NVS_NAMESPACE       "microlink"
#define NVS_KEY_MACHINE_PRI "machine_pri"
#define NVS_KEY_MACHINE_PUB "machine_pub"
#define NVS_KEY_WG_PRI      "wg_private"
#define NVS_KEY_WG_PUB      "wg_public"
#define NVS_KEY_DISCO_PRI   "disco_pri"
#define NVS_KEY_DISCO_PUB   "disco_pub"

/* X25519 from x25519.h */
#include "x25519.h"

/* ============================================================================
 * Key Management (loaded once at init, read-only after)
 * ========================================================================== */

static void generate_keypair(uint8_t *private_key, uint8_t *public_key) {
    esp_fill_random(private_key, 32);
    private_key[0] &= 248;
    private_key[31] &= 127;
    private_key[31] |= 64;
    x25519_base(public_key, private_key, 1);
}

static esp_err_t load_or_generate_keys(microlink_t *ml) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (%d), generating ephemeral keys", err);
        generate_keypair(ml->machine_private_key, ml->machine_public_key);
        generate_keypair(ml->wg_private_key, ml->wg_public_key);
        generate_keypair(ml->disco_private_key, ml->disco_public_key);
        return ESP_OK;
    }

    size_t key_len = 32;
    bool need_save = false;

    /* Machine key */
    if (nvs_get_blob(nvs, NVS_KEY_MACHINE_PRI, ml->machine_private_key, &key_len) != ESP_OK) {
        generate_keypair(ml->machine_private_key, ml->machine_public_key);
        need_save = true;
        ESP_LOGI(TAG, "Generated new machine key");
    } else {
        key_len = 32;
        nvs_get_blob(nvs, NVS_KEY_MACHINE_PUB, ml->machine_public_key, &key_len);
    }

    /* WireGuard key */
    key_len = 32;
    if (nvs_get_blob(nvs, NVS_KEY_WG_PRI, ml->wg_private_key, &key_len) != ESP_OK) {
        generate_keypair(ml->wg_private_key, ml->wg_public_key);
        need_save = true;
        ESP_LOGI(TAG, "Generated new WireGuard key");
    } else {
        key_len = 32;
        nvs_get_blob(nvs, NVS_KEY_WG_PUB, ml->wg_public_key, &key_len);
    }

    /* DISCO key */
    key_len = 32;
    if (nvs_get_blob(nvs, NVS_KEY_DISCO_PRI, ml->disco_private_key, &key_len) != ESP_OK) {
        generate_keypair(ml->disco_private_key, ml->disco_public_key);
        need_save = true;
        ESP_LOGI(TAG, "Generated new DISCO key");
    } else {
        key_len = 32;
        nvs_get_blob(nvs, NVS_KEY_DISCO_PUB, ml->disco_public_key, &key_len);
    }

    if (need_save) {
        nvs_set_blob(nvs, NVS_KEY_MACHINE_PRI, ml->machine_private_key, 32);
        nvs_set_blob(nvs, NVS_KEY_MACHINE_PUB, ml->machine_public_key, 32);
        nvs_set_blob(nvs, NVS_KEY_WG_PRI, ml->wg_private_key, 32);
        nvs_set_blob(nvs, NVS_KEY_WG_PUB, ml->wg_public_key, 32);
        nvs_set_blob(nvs, NVS_KEY_DISCO_PRI, ml->disco_private_key, 32);
        nvs_set_blob(nvs, NVS_KEY_DISCO_PUB, ml->disco_public_key, 32);
        nvs_commit(nvs);
        ESP_LOGI(TAG, "Keys saved to NVS");
    } else {
        ESP_LOGI(TAG, "Keys loaded from NVS");
    }

    nvs_close(nvs);
    return ESP_OK;
}

/* ============================================================================
 * cJSON PSRAM Hooks
 * ========================================================================== */

static void *cjson_psram_malloc(size_t size) {
    return ml_psram_malloc(size);
}

/* ============================================================================
 * WiCAN: PSRAM-stack task helper (internal-RAM budget)
 *
 * Creates a static task with its stack in PSRAM and its TCB in internal
 * RAM. For microlink's TLS/socket/crypto tasks (net_io/derp_tx/coord)
 * that never touch flash — moving 34 KB of stacks off the scarce
 * internal heap. Self-deletion via vTaskDelete(NULL) is safe for static
 * tasks (FreeRTOS just marks them done); the PSRAM stacks are freed by
 * ml_free_psram_task_stacks() in microlink_destroy after the join.
 * ========================================================================== */
#define ML_PSRAM_TASKS 3
static StaticTask_t s_ml_tcb[ML_PSRAM_TASKS];   /* internal: FreeRTOS TCB */
static StackType_t *s_ml_stack[ML_PSRAM_TASKS]; /* PSRAM stack buffers    */
/* Slot whose task failed to join in microlink_stop(): the task may still
 * be alive on this stack/TCB, so the stack is never freed and the slot is
 * never reused (see notes/BUG_TS_DNS_FOUND_CRASH.md). Only reboot clears. */
static bool s_ml_slot_leaked[ML_PSRAM_TASKS];

static bool ml_start_psram_task(TaskFunction_t fn, const char *name,
                                uint32_t stack_bytes, void *arg,
                                UBaseType_t prio, TaskHandle_t *out,
                                BaseType_t core, int slot) {
    if (s_ml_slot_leaked[slot]) {
        ESP_LOGE(TAG, "PSRAM task slot %d poisoned by an unjoined task; "
                      "refusing to reuse (reboot to recover)", slot);
        return false;
    }

    StackType_t *stk = heap_caps_malloc(stack_bytes,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stk == NULL) {
        return false;
    }

    TaskHandle_t h = xTaskCreateStaticPinnedToCore(
        fn, name, stack_bytes / sizeof(StackType_t), arg, prio,
        stk, &s_ml_tcb[slot], core);
    if (h == NULL) {
        heap_caps_free(stk);
        return false;
    }

    s_ml_stack[slot] = stk;
    *out = h;
    return true;
}

static void ml_free_psram_task_stacks(void) {
    for (int i = 0; i < ML_PSRAM_TASKS; i++) {
        if (s_ml_stack[i] != NULL && !s_ml_slot_leaked[i]) {
            heap_caps_free(s_ml_stack[i]);
            s_ml_stack[i] = NULL;
        }
    }
}

/* Task-exit handshake (see microlink_internal.h). Parking instead of
 * vTaskDelete(NULL) lets microlink_stop() delete the task while it is
 * provably NOT running, which makes freeing its PSRAM stack safe. */
void ml_task_exit(microlink_t *ml, EventBits_t exit_bit) {
    xEventGroupSetBits(ml->events, exit_bit);
    for (;;) {
        vTaskSuspend(NULL);
    }
}

/* ============================================================================
 * Factory Reset
 * ========================================================================== */

esp_err_t microlink_factory_reset(void) {
    esp_err_t err;

    /* Erase key namespace */
    nvs_handle_t nvs;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Factory reset: keys erased");
    }

    /* Erase cached peers */
    ml_peer_nvs_init();
    ml_peer_nvs_clear();
    ml_peer_nvs_deinit();

    ESP_LOGI(TAG, "Factory reset complete");
    return ESP_OK;
}

/* ============================================================================
 * Public API
 * ========================================================================== */

microlink_t *microlink_init(const microlink_config_t *config) {
    if (!config || !config->auth_key) {
        ESP_LOGE(TAG, "Invalid config: auth_key required");
        return NULL;
    }

    /* Route cJSON to PSRAM */
    cJSON_Hooks hooks = {
        .malloc_fn = cjson_psram_malloc,
        .free_fn = free
    };
    cJSON_InitHooks(&hooks);

    /* Allocate context from PSRAM */
    microlink_t *ml = ml_psram_calloc(1, sizeof(microlink_t));
    if (!ml) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return NULL;
    }

    /* Copy config */
    ml->config = *config;
    if (ml->config.max_peers == 0) ml->config.max_peers = ML_MAX_PEERS;
    if (ml->config.max_peers > ML_MAX_PEERS) ml->config.max_peers = ML_MAX_PEERS;
    ml->config.enable_derp = true;  /* Always need DERP for relay */

    ml->state = ML_STATE_IDLE;
    ml->coord_sock = -1;
    ml->disco_sock4 = -1;
    ml->disco_sock6 = -1;
    ml->stun_sock = -1;
    ml->stun_sock6 = -1;
    ml->derp.sockfd = -1;

    /* Resolve timing (0 = use defaults from #defines) */
    ml->t_disco_heartbeat_ms = ml->config.disco_heartbeat_ms ? ml->config.disco_heartbeat_ms : ML_DISCO_HEARTBEAT_MS;
    ml->t_stun_interval_ms = ml->config.stun_interval_ms ? ml->config.stun_interval_ms : ML_STUN_RESTUN_INTERVAL_MS;
    ml->t_ctrl_watchdog_ms = ml->config.ctrl_watchdog_ms ? ml->config.ctrl_watchdog_ms : ML_CTRL_WATCHDOG_MS;

    /* Apply Kconfig priority peer if set and app didn't provide one */
    if (ml->config.priority_peer_ip == 0 && strlen(CONFIG_ML_PRIORITY_PEER_IP) > 0) {
        ml->config.priority_peer_ip = microlink_parse_ip(CONFIG_ML_PRIORITY_PEER_IP);
        if (ml->config.priority_peer_ip) {
            ESP_LOGI(TAG, "Priority peer from Kconfig: %s", CONFIG_ML_PRIORITY_PEER_IP);
        }
    }

    /* Load or generate persistent keys */
    if (load_or_generate_keys(ml) != ESP_OK) {
        free(ml);
        return NULL;
    }

    /* Initialize peer NVS cache */
    ml_peer_nvs_init();

    /* Initialize HTTP config server (loads NVS peer allowlist + settings) */
    /* WiCAN: coordination-server override from the caller's config
     * (Headscale/Ionscale) — takes effect before any NVS override */
    ml->ctrl_port = 80;
    if (config->control_url != NULL && config->control_url[0] != '\0') {
        strncpy(ml->ctrl_host, config->control_url, sizeof(ml->ctrl_host) - 1);
        char *colon = strrchr(ml->ctrl_host, ':');
        if (colon != NULL) {
            *colon = '\0';
            int pnum = atoi(colon + 1);
            if (pnum > 0 && pnum < 65536) {
                ml->ctrl_port = pnum;
            }
        }
        ESP_LOGI(TAG, "Control plane from config: %s:%d",
                 ml->ctrl_host, ml->ctrl_port);
    }

    ml->config_httpd = ml_config_httpd_init();

    /* Override config with NVS-saved settings (web UI save → restart flow).
     * NVS settings take priority over Kconfig defaults.  Strings are copied
     * into ml->nvs_* buffers so the const char* pointers remain valid. */
    if (ml->config_httpd) {
        const char *nvs_auth = ml_config_get_auth_key(ml->config_httpd);
        if (nvs_auth) {
            strncpy(ml->nvs_auth_key, nvs_auth, sizeof(ml->nvs_auth_key) - 1);
            ml->config.auth_key = ml->nvs_auth_key;
            ESP_LOGI(TAG, "Auth key overridden from NVS (len=%d)", (int)strlen(nvs_auth));
        }
        /* Device name: full name takes priority, then prefix+MAC, then Kconfig */
        const char *nvs_full_name = ml_config_get_device_name_full(ml->config_httpd);
        if (nvs_full_name) {
            /* Full custom hostname (e.g. "sensor-tailscale") */
            strncpy(ml->nvs_device_name, nvs_full_name, sizeof(ml->nvs_device_name) - 1);
            ml->config.device_name = ml->nvs_device_name;
            ESP_LOGI(TAG, "Device name from NVS (full): %s", ml->nvs_device_name);
        } else {
            const char *nvs_prefix = ml_config_get_device_prefix(ml->config_httpd);
            if (nvs_prefix) {
                /* Device name = prefix + MAC suffix (e.g. "sensor-a1b2c3") */
                uint8_t mac[6];
                esp_read_mac(mac, ESP_MAC_WIFI_STA);
                snprintf(ml->nvs_device_name, sizeof(ml->nvs_device_name),
                         "%s-%02x%02x%02x", nvs_prefix, mac[3], mac[4], mac[5]);
                ml->config.device_name = ml->nvs_device_name;
                ESP_LOGI(TAG, "Device name from NVS (prefix): %s", ml->nvs_device_name);
            }
        }

        /* v2 overrides */
        uint8_t nvs_max = ml_config_get_max_peers(ml->config_httpd);
        if (nvs_max > 0 && nvs_max <= ML_MAX_PEERS) {
            ml->config.max_peers = nvs_max;
            ESP_LOGI(TAG, "Max peers overridden from NVS: %d", nvs_max);
        }
        uint16_t nvs_hb = ml_config_get_disco_heartbeat_ms(ml->config_httpd);
        if (nvs_hb > 0 && nvs_hb <= 60000) {
            ml->config.disco_heartbeat_ms = nvs_hb;
            ml->t_disco_heartbeat_ms = nvs_hb;
            ESP_LOGI(TAG, "DISCO heartbeat overridden from NVS: %dms", nvs_hb);
        }
        uint32_t nvs_pip = ml_config_get_priority_peer_ip(ml->config_httpd);
        if (nvs_pip > 0) {
            ml->config.priority_peer_ip = nvs_pip;
            char pip_str[16];
            microlink_ip_to_str(nvs_pip, pip_str);
            ESP_LOGI(TAG, "Priority peer overridden from NVS: %s", pip_str);
        }
        const char *nvs_ctrl = ml_config_get_ctrl_host(ml->config_httpd);
        if (nvs_ctrl) {
            strncpy(ml->ctrl_host, nvs_ctrl, sizeof(ml->ctrl_host) - 1);
            ESP_LOGI(TAG, "Control plane overridden from NVS: %s", ml->ctrl_host);
        }
        ml->debug_flags = ml_config_get_debug_flags(ml->config_httpd);
        if (ml->debug_flags) {
            ESP_LOGI(TAG, "Debug flags from NVS: 0x%02x", ml->debug_flags);
        }
    }

    /* Create event group */
    ml->events = xEventGroupCreate();
    if (!ml->events) {
        ESP_LOGE(TAG, "Failed to create event group");
        free(ml);
        return NULL;
    }

    /* Create queues */
    ml->derp_tx_queue = xQueueCreate(ML_DERP_TX_QUEUE_DEPTH, sizeof(ml_derp_tx_item_t));
    ml->disco_rx_queue = xQueueCreate(ML_DISCO_RX_QUEUE_DEPTH, sizeof(ml_rx_packet_t));
    ml->wg_rx_queue = xQueueCreate(ML_WG_RX_QUEUE_DEPTH, sizeof(ml_rx_packet_t));
    ml->stun_rx_queue = xQueueCreate(ML_STUN_RX_QUEUE_DEPTH, sizeof(ml_rx_packet_t));
    ml->coord_cmd_queue = xQueueCreate(ML_COORD_CMD_QUEUE_DEPTH, sizeof(ml_coord_cmd_t));
    ml->peer_update_queue = xQueueCreate(ML_PEER_UPDATE_QUEUE_DEPTH, sizeof(ml_peer_update_t *));

    if (!ml->derp_tx_queue || !ml->disco_rx_queue || !ml->wg_rx_queue ||
        !ml->stun_rx_queue || !ml->coord_cmd_queue || !ml->peer_update_queue) {
        ESP_LOGE(TAG, "Failed to create queues");
        microlink_destroy(ml);
        return NULL;
    }

    ESP_LOGI(TAG, "MicroLink v2 initialized (max_peers=%d)", ml->config.max_peers);
    return ml;
}

esp_err_t microlink_start(microlink_t *ml) {
    if (!ml) return ESP_ERR_INVALID_ARG;
    if (ml->state != ML_STATE_IDLE) {
        ESP_LOGW(TAG, "Already started (state=%d)", ml->state);
        return ESP_ERR_INVALID_STATE;
    }

    ml->state = ML_STATE_WIFI_WAIT;

    /* Set WiFi TX power if configured */
    if (ml->config.wifi_tx_power_dbm > 0) {
        int8_t power_quarter_dbm = ml->config.wifi_tx_power_dbm * 4;
        esp_wifi_set_max_tx_power(power_quarter_dbm);
        ESP_LOGI(TAG, "WiFi TX power set to %d dBm", ml->config.wifi_tx_power_dbm);
    }

#ifdef CONFIG_ML_ZERO_COPY_WG
    /* Zero-copy mode: raw lwIP PCB replaces BSD socket for DISCO/WG UDP.
     * WG packets go directly to wireguardif_network_rx() from tcpip_thread.
     * DISCO packets go to SPSC ring buffer, STUN to existing queue. */
    if (ml_zerocopy_init(ml) != ESP_OK) {
        ESP_LOGE(TAG, "Zero-copy init failed, falling back to BSD socket");
        goto bsd_socket_fallback;
    }
    ESP_LOGI(TAG, "Zero-copy WG mode active (high-throughput)");
    goto skip_bsd_socket;

bsd_socket_fallback:
#endif
    /* Create DISCO/magicsock UDP socket (port 51820 = WireGuard standard)
     * This is the single socket for ALL direct UDP traffic: DISCO pings/pongs,
     * CallMeMaybe probes, and WireGuard encrypted data.
     * Matches v1 microlink_disco_init() and tailscale's pconn4. */
    ml->disco_sock4 = ml_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ml->disco_sock4 >= 0) {
        struct sockaddr_in bind_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(51820),
            .sin_addr.s_addr = INADDR_ANY,
        };
        if (ml_bind(ml->disco_sock4, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            ESP_LOGW(TAG, "Failed to bind port 51820 (errno=%d), trying ephemeral", errno);
            bind_addr.sin_port = 0;
            ml_bind(ml->disco_sock4, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
        }

        /* Mark packets as DSCP 46 (Expedited Forwarding) → WMM AC_VO.
         * WiFi APs with WMM use shorter contention windows for voice-priority
         * traffic, reducing jitter on WireGuard/DISCO UDP packets. */
        int tos = 0xB8;  /* DSCP 46 = EF, TOS byte = 46 << 2 = 184 */
        setsockopt(ml->disco_sock4, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

        /* Set non-blocking for select() in net_io */
        int flags = ml_fcntl(ml->disco_sock4, F_GETFL, 0);
        ml_fcntl(ml->disco_sock4, F_SETFL, flags | O_NONBLOCK);

        /* Record the actual bound port (getsockname not wrapped — AT sockets use stored port) */
        struct sockaddr_in local_addr;
        socklen_t addr_len = sizeof(local_addr);
        getsockname(ml->disco_sock4, (struct sockaddr *)&local_addr, &addr_len);
        ml->disco_local_port = ntohs(local_addr.sin_port);
        ESP_LOGI(TAG, "DISCO/magicsock UDP socket bound to port %d", ml->disco_local_port);
    } else {
        ESP_LOGE(TAG, "Failed to create DISCO socket: errno=%d", errno);
    }
#ifdef CONFIG_ML_ZERO_COPY_WG
skip_bsd_socket:
    ;
#endif

    /* Create tasks.
     * WiCAN (internal-RAM budget): net_io/derp_tx/coord run PSRAM
     * stacks (34 KB moved off internal) — they do TLS/sockets/crypto
     * only, never flash, so the §2 corollary allows it. Their TCBs stay
     * internal (the scheduler touches TCBs). wg_mgr KEEPS an internal
     * stack: it writes the peer cache to NVS (flash), which a PSRAM
     * stack forbids. Self-deleting static tasks are safe — the memory
     * is ours; the PSRAM stacks are freed in microlink_destroy. */
    BaseType_t ret;

    if (!ml_start_psram_task(ml_net_io_task, "ml_net_io",
                             ML_TASK_NET_IO_STACK, ml, ML_TASK_NET_IO_PRIO,
                             &ml->net_io_task, ML_TASK_NET_IO_CORE, 0)) {
        ESP_LOGE(TAG, "Failed to create net_io task");
        return ESP_FAIL;
    }

    if (!ml_start_psram_task(ml_derp_tx_task, "ml_derp_tx",
                             ML_TASK_DERP_TX_STACK, ml, ML_TASK_DERP_TX_PRIO,
                             &ml->derp_tx_task, ML_TASK_DERP_TX_CORE, 1)) {
        ESP_LOGE(TAG, "Failed to create derp_tx task");
        return ESP_FAIL;
    }

    if (!ml_start_psram_task(ml_coord_task, "ml_coord",
                             ML_TASK_COORD_STACK, ml, ML_TASK_COORD_PRIO,
                             &ml->coord_task, ML_TASK_COORD_CORE, 2)) {
        ESP_LOGE(TAG, "Failed to create coord task");
        return ESP_FAIL;
    }

    /* wg_mgr: INTERNAL stack (NVS peer-cache writes) */
    ret = xTaskCreatePinnedToCore(ml_wg_mgr_task, "ml_wg_mgr", ML_TASK_WG_MGR_STACK,
                                   ml, ML_TASK_WG_MGR_PRIO, &ml->wg_mgr_task, ML_TASK_WG_MGR_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create wg_mgr task");
        return ESP_FAIL;
    }

    /* WiFi is expected to be connected before microlink_start() is called.
     * Signal the event so coord/wg_mgr tasks proceed immediately. */
    xEventGroupSetBits(ml->events, ML_EVT_WIFI_CONNECTED);

    /* Signal coord task to start connecting */
    ml_coord_cmd_t cmd = ML_CMD_CONNECT;
    xQueueSend(ml->coord_cmd_queue, &cmd, 0);

    /* Start HTTP config server (binds port 80, serves config page + REST API) */
    if (ml->config_httpd) {
        ml_config_httpd_start(ml->config_httpd, ml);
    }

    ESP_LOGI(TAG, "All tasks started");
    return ESP_OK;
}

/* Join budget: must comfortably exceed the longest blocking call a worker
 * can sit in after ML_EVT_SHUTDOWN_REQUEST — the worst is a plain-lwip
 * getaddrinfo() (no timeout knob), bounded by the lwip DNS retry schedule
 * at ~16 s over a dead uplink. */
#define ML_TASK_JOIN_TIMEOUT_MS 30000

esp_err_t microlink_stop(microlink_t *ml) {
    if (!ml) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Stopping...");
    xEventGroupSetBits(ml->events, ML_EVT_SHUTDOWN_REQUEST);

    /* Join the tasks for real. Each task signals its exit bit and parks in
     * ml_task_exit(); once a task is provably parked we delete it here.
     * The old blind 3 s grace period was NOT a join: a worker inside a
     * blocking getaddrinfo() outlived it, and freeing its PSRAM stack then
     * left the lwip DNS found-callback pointing into freed memory
     * (StoreProhibited in dns_tmr — notes/BUG_TS_DNS_FOUND_CRASH.md). */
    const struct {
        TaskHandle_t *handle;
        EventBits_t bit;
        int psram_slot; /* -1 = dynamic stack (freed by the kernel) */
        const char *name;
    } join[] = {
        { &ml->net_io_task,  ML_EVT_NET_IO_EXITED,  0,  "net_io"  },
        { &ml->derp_tx_task, ML_EVT_DERP_TX_EXITED, 1,  "derp_tx" },
        { &ml->coord_task,   ML_EVT_COORD_EXITED,   2,  "coord"   },
        { &ml->wg_mgr_task,  ML_EVT_WG_MGR_EXITED,  -1, "wg_mgr"  },
    };

    EventBits_t want = 0;
    for (size_t i = 0; i < sizeof(join) / sizeof(join[0]); i++) {
        if (*join[i].handle != NULL) {
            want |= join[i].bit;
        }
    }

    EventBits_t got = 0;
    if (want != 0) {
        got = xEventGroupWaitBits(ml->events, want, pdFALSE, pdTRUE,
                                  pdMS_TO_TICKS(ML_TASK_JOIN_TIMEOUT_MS));
    }

    for (size_t i = 0; i < sizeof(join) / sizeof(join[0]); i++) {
        TaskHandle_t h = *join[i].handle;
        if (h == NULL) {
            continue;
        }
        if (got & join[i].bit) {
            /* Exit bit set; wait for the final park (the window between
             * xEventGroupSetBits and vTaskSuspend in ml_task_exit). */
            int spins = 0;
            while (eTaskGetState(h) != eSuspended && ++spins < 100) {
                vTaskDelay(1);
            }
            if (eTaskGetState(h) == eSuspended) {
                vTaskDelete(h); /* not running: stack is now truly idle */
                *join[i].handle = NULL;
                continue;
            }
        }
        /* Task never signalled (or never parked): it may still be blocked
         * in lwip with stack-resident state registered in the DNS table.
         * Leak its stack and everything it can reach — never free. */
        ESP_LOGE(TAG, "Task %s failed to join within %d ms — leaking its "
                      "stack and shared microlink resources",
                 join[i].name, ML_TASK_JOIN_TIMEOUT_MS);
        if (join[i].psram_slot >= 0) {
            s_ml_slot_leaked[join[i].psram_slot] = true;
        }
        ml->tasks_leaked = true;
        *join[i].handle = NULL;
    }

    /* Stop HTTP config server */
    if (ml->config_httpd) {
        ml_config_httpd_stop(ml->config_httpd);
    }

    /* Clean up zero-copy PCB if active */
#ifdef CONFIG_ML_ZERO_COPY_WG
    ml_zerocopy_deinit(ml);
#endif

    /* Close sockets */
    if (ml->disco_sock4 >= 0) { ml_close_sock(ml->disco_sock4); ml->disco_sock4 = -1; }
    if (ml->stun_sock >= 0) { ml_close_sock(ml->stun_sock); ml->stun_sock = -1; }
    if (ml->stun_sock6 >= 0) { ml_close_sock(ml->stun_sock6); ml->stun_sock6 = -1; }

    ml->state = ML_STATE_IDLE;
    ESP_LOGI(TAG, "Stopped");
    return ESP_OK;
}

void microlink_destroy(microlink_t *ml) {
    if (!ml) return;

    microlink_stop(ml);

    if (ml->tasks_leaked) {
        /* A task failed to join — it may still wake up and touch the
         * event group, queues, sockets, or its own stack. Leak the whole
         * microlink instance rather than freeing memory under a live
         * task (notes/BUG_TS_DNS_FOUND_CRASH.md). ~40 KB lost until
         * reboot on a path that should never be hit. */
        ESP_LOGE(TAG, "Leaking microlink instance: unjoined task(s)");
        return;
    }

    /* WiCAN: PSRAM task stacks are ours to free — safe now that
     * microlink_stop() joined and deleted the (parked) tasks */
    ml_free_psram_task_stacks();

    /* Deinitialize peer NVS */
    ml_peer_nvs_deinit();

    /* Deinitialize HTTP config server */
    if (ml->config_httpd) {
        ml_config_httpd_deinit(ml->config_httpd);
        ml->config_httpd = NULL;
    }

    /* Delete queues */
    if (ml->derp_tx_queue) vQueueDelete(ml->derp_tx_queue);
    if (ml->disco_rx_queue) vQueueDelete(ml->disco_rx_queue);
    if (ml->wg_rx_queue) vQueueDelete(ml->wg_rx_queue);
    if (ml->stun_rx_queue) vQueueDelete(ml->stun_rx_queue);
    if (ml->coord_cmd_queue) vQueueDelete(ml->coord_cmd_queue);
    if (ml->peer_update_queue) vQueueDelete(ml->peer_update_queue);

    /* Delete event group */
    if (ml->events) vEventGroupDelete(ml->events);

    /* WiCAN: free the long-poll map reassembly buffers */
    if (ml->map_stream_buf) {
        free(ml->map_stream_buf);
        ml->map_stream_buf = NULL;
    }
    if (ml->h2_acc_buf) {
        free(ml->h2_acc_buf);
        ml->h2_acc_buf = NULL;
    }

    /* Clear keys from memory */
    memset(ml->machine_private_key, 0, 32);
    memset(ml->wg_private_key, 0, 32);
    memset(ml->disco_private_key, 0, 32);

    free(ml);
    ESP_LOGI(TAG, "Destroyed");
}

/* ============================================================================
 * State Queries
 * ========================================================================== */

microlink_state_t microlink_get_state(const microlink_t *ml) {
    return ml ? ml->state : ML_STATE_IDLE;
}

bool microlink_is_connected(const microlink_t *ml) {
    return ml && ml->state == ML_STATE_CONNECTED;
}

uint32_t microlink_get_vpn_ip(const microlink_t *ml) {
    return ml ? ml->vpn_ip : 0;
}

int microlink_get_peer_count(const microlink_t *ml) {
    return ml ? ml->peer_count : 0;
}

esp_err_t microlink_get_peer_info(const microlink_t *ml, int index, microlink_peer_info_t *info) {
    if (!ml || !info || index < 0 || index >= ml->peer_count) {
        return ESP_ERR_INVALID_ARG;
    }
    const ml_peer_t *p = &ml->peers[index];
    if (!p->active) {
        /* Pruned slot inside peer_count (removal only tail-compacts):
         * reporting it resurrects ghost peers in /api/vpn */
        return ESP_ERR_NOT_FOUND;
    }
    info->vpn_ip = p->vpn_ip;
    strncpy(info->hostname, p->hostname, sizeof(info->hostname) - 1);
    memcpy(info->public_key, p->public_key, 32);
    info->online = p->active;
    info->direct_path = p->has_direct_path;
    return ESP_OK;
}

/* ============================================================================
 * Send API
 * ========================================================================== */

esp_err_t microlink_send(microlink_t *ml, uint32_t dest_vpn_ip,
                          const uint8_t *data, size_t len) {
    if (!ml || !data || len == 0 || len > 1400) return ESP_ERR_INVALID_ARG;
    if (ml->state != ML_STATE_CONNECTED) return ESP_ERR_INVALID_STATE;

    /* Find peer by VPN IP */
    for (int i = 0; i < ml->peer_count; i++) {
        if (ml->peers[i].vpn_ip == dest_vpn_ip && ml->peers[i].active) {
            /* TODO: Route through WireGuard tunnel */
            /* For now, queue via DERP as fallback */
            return ml_derp_queue_send(ml, ml->peers[i].public_key, data, len);
        }
    }
    return ESP_ERR_NOT_FOUND;
}

/* ============================================================================
 * Callbacks
 * ========================================================================== */

void microlink_set_state_callback(microlink_t *ml, microlink_state_cb_t cb, void *user_data) {
    if (ml) { ml->state_cb = cb; ml->state_cb_data = user_data; }
}

void microlink_set_peer_callback(microlink_t *ml, microlink_peer_cb_t cb, void *user_data) {
    if (ml) { ml->peer_cb = cb; ml->peer_cb_data = user_data; }
}

void microlink_set_data_callback(microlink_t *ml, microlink_data_cb_t cb, void *user_data) {
    if (ml) { ml->data_cb = cb; ml->data_cb_data = user_data; }
}

/* ============================================================================
 * Utilities
 * ========================================================================== */

void microlink_ip_to_str(uint32_t ip, char *buf) {
    snprintf(buf, 16, "%lu.%lu.%lu.%lu",
             (unsigned long)((ip >> 24) & 0xFF),
             (unsigned long)((ip >> 16) & 0xFF),
             (unsigned long)((ip >> 8) & 0xFF),
             (unsigned long)(ip & 0xFF));
}

uint32_t microlink_parse_ip(const char *ip_str) {
    if (!ip_str) return 0;
    unsigned int a, b, c, d;
    if (sscanf(ip_str, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return 0;
    if (a > 255 || b > 255 || c > 255 || d > 255) return 0;
    return (a << 24) | (b << 16) | (c << 8) | d;
}

const char *microlink_default_device_name(void) {
    static char name[48] = {0};
    if (name[0] == 0) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        /* Use ML_DEVICE_NAME as prefix if configured, otherwise "esp32" */
        const char *prefix = CONFIG_ML_DEVICE_NAME;
        if (!prefix || !prefix[0]) prefix = "esp32";
        snprintf(name, sizeof(name), "%s-%02x%02x%02x", prefix, mac[3], mac[4], mac[5]);
    }
    return name;
}

const char *microlink_imei_device_name(void) {
#ifdef CONFIG_ML_ENABLE_CELLULAR
    static char name[48] = {0};  /* prefix + "-" + 15-digit IMEI + null */
    const char *imei = ml_cellular_get_imei();
    if (imei && imei[0]) {
        const char *prefix = CONFIG_ML_DEVICE_NAME;
        if (!prefix || !prefix[0]) prefix = "esp32";
        snprintf(name, sizeof(name), "%s-%s", prefix, imei);
        return name;
    }
#endif
    return NULL;
}

uint64_t ml_get_time_ms(void) {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* ============================================================================
 * MagicDNS — Resolve tailnet hostnames against peer list
 * ========================================================================== */

/* Case-insensitive string compare (limited to len bytes) */
static int strncasecmp_local(const char *a, const char *b, size_t len) {
    for (size_t i = 0; i < len; i++) {
        char ca = a[i], cb = b[i];
        if (ca >= 'A' && ca <= 'Z') ca += 32;
        if (cb >= 'A' && cb <= 'Z') cb += 32;
        if (ca != cb) return ca - cb;
        if (ca == '\0') return 0;
    }
    return 0;
}

uint32_t microlink_resolve(const microlink_t *ml, const char *hostname) {
    if (!ml || !hostname || hostname[0] == '\0') return 0;

    size_t query_len = strlen(hostname);

    for (int i = 0; i < ml->peer_count; i++) {
        const ml_peer_t *p = &ml->peers[i];
        if (!p->active || p->hostname[0] == '\0') continue;

        /* 1. Exact match (case-insensitive) */
        if (strncasecmp_local(p->hostname, hostname, sizeof(p->hostname)) == 0) {
            return p->vpn_ip;
        }

        /* 2. Prefix match: query "npc1" matches peer "npc1.tail12345.ts.net"
         * The query must match up to the first '.' in the peer hostname. */
        const char *dot = strchr(p->hostname, '.');
        if (dot) {
            size_t short_len = (size_t)(dot - p->hostname);
            if (query_len == short_len &&
                strncasecmp_local(p->hostname, hostname, short_len) == 0) {
                return p->vpn_ip;
            }
        }
    }

    return 0;  /* Not found */
}
