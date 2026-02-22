#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif.h"

#include "modem_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    modem_mgr_config_t modem;

    const char *apn;
    const char *user;
    const char *pass;

    bool init_modem_manager;

    int baud_rate;
    bool hw_flow_control;

    // If true, the module will call usb_dev_ethernet_enable_sharing() when PPP gets an IP.
    // This is optional glue for "LTE sharing over USB NCM".
    bool enable_usb_ncm_sharing;

    uint32_t connect_timeout_ms;
} lte_upstream_pppos_config_t;

// Snapshot of LTE modem status — filled by lte_upstream_pppos_get_status().
typedef struct {
    bool  valid;             // true once the monitor task has completed at least one poll
    int   rssi;              // AT+CSQ raw value (0-31, 99=unknown)
    int   rssi_dbm;          // dBm: rssi*2-113 (meaningless when rssi==99)
    int   ber;               // bit-error-rate class (0-7, 99=unknown)
    bool  attached;          // network attachment state (CEREG/CREG)
    bool  ppp_connected;     // true after IP_EVENT_PPP_GOT_IP, false after LOST_IP
    esp_ip4_addr_t current_ip; // last IP assigned by PPP; zeroed when lost
    char  operator_name[64]; // operator name from AT+COPS (empty until first query)
    int   operator_act;      // access technology (7=LTE, 2=UTRAN, etc.)
} lte_status_t;

esp_err_t lte_upstream_pppos_start(const lte_upstream_pppos_config_t *cfg);

esp_err_t lte_upstream_pppos_wait_for_ip(uint32_t timeout_ms);

esp_err_t lte_upstream_pppos_stop(void);

// Copy the latest modem status into *out. Thread-safe (spinlock-protected).
// Returns ESP_ERR_INVALID_STATE if the monitor has not yet run.
esp_err_t lte_upstream_pppos_get_status(lte_status_t *out);

// Register the `lte` console command. Called by usb_cli_console.
void lte_upstream_pppos_console_register(void);

#ifdef __cplusplus
}
#endif
