#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

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

esp_err_t lte_upstream_pppos_start(const lte_upstream_pppos_config_t *cfg);

esp_err_t lte_upstream_pppos_wait_for_ip(uint32_t timeout_ms);

esp_err_t lte_upstream_pppos_stop(void);

#ifdef __cplusplus
}
#endif
