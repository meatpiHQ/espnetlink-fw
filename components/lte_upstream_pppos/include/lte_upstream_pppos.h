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
    const char *pin;   // SIM PIN (NULL if PIN lock is disabled, which is the default for most IoT SIMs)

    bool init_modem_manager;

    int baud_rate;
    bool hw_flow_control;

    // If true, the module will call usb_dev_ethernet_enable_sharing() when PPP gets an IP.
    // This is optional glue for "LTE sharing over USB NCM".
    bool enable_usb_ncm_sharing;

    uint32_t connect_timeout_ms;
    uint32_t reg_timeout_ms;       /* LTE registration wait timeout in ms (0 = default 120 s) */

    /* If LTE registration times out, optionally issue a soft modem reboot.
     * AT+CFUN=1,1 triggers a full modem restart (Quectel BG95 and many others).
     * When enabled, the component will send the command once and then fail the
     * start attempt so the supervisor can retry from a clean state.
     */
    bool     reboot_on_reg_timeout;  /* default: false */
    uint32_t reboot_wait_ms;         /* delay after CFUN (0 = default 15000 ms) */

    bool use_cmux;                 /* true (default) = CMUX (PPP+AT), false = direct DATA mode
                                    * Direct mode gives higher throughput but AT commands are
                                    * unavailable while PPP is running (no signal monitor). */

    /* --- USB transport (alternative to UART) --------------------------------
     * When use_usb = true the component opens the modem's native USB CDC-ACM
     * port and uses it as the DTE instead of UART.  All other config fields
     * (apn, user, pass, pin, reg_timeout_ms, use_cmux, …) still apply.
     * The modem still needs to be powered on – modem_manager (UART) handles
     * that before the USB DTE is created.
     *
     * BG95 defaults:  vid=0x2C7C  pid=0x0095  usb_interface=2
     * Leave vid/pid/usb_interface at 0 to use those defaults.
     */
    bool     use_usb;            /* true = use BG95 USB CDC-ACM port instead of UART */
    uint16_t usb_vid;            /* USB Vendor ID  (0 = use 0x2C7C)  */
    uint16_t usb_pid;            /* USB Product ID (0 = use 0x0095)  */
    uint8_t  usb_interface;      /* CDC-ACM interface for AT/data port (0 = use 2) */
    int8_t   usb_sel_gpio;       /* GPIO to control USB MUX (-1 = not used)  */
    bool     usb_sel_level;      /* Logic level to assert on usb_sel_gpio    */
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
    char  network_type[32];  // human-readable from AT+QNWINFO: "CAT-M1", "NB-IoT", "LTE", "HSPA+", "GSM", etc.
} lte_status_t;

esp_err_t lte_upstream_pppos_start(const lte_upstream_pppos_config_t *cfg);

esp_err_t lte_upstream_pppos_wait_for_ip(uint32_t timeout_ms);

esp_err_t lte_upstream_pppos_stop(void);

// Copy the latest modem status into *out. Thread-safe (spinlock-protected).
// Returns ESP_ERR_INVALID_STATE if the monitor has not yet run.
esp_err_t lte_upstream_pppos_get_status(lte_status_t *out);

/**
 * Send a raw AT command to the modem and return the response.
 *
 * Requires CMUX mode (use_cmux=true) so an AT channel is available
 * while PPP is running.  Thread-safe.
 *
 * @param cmd        AT command string (e.g. "AT+QLTS=2")
 * @param resp       Buffer for response text (NUL-terminated)
 * @param resp_len   Size of resp buffer
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if DCE unavailable.
 */
esp_err_t lte_upstream_pppos_send_at(const char *cmd, char *resp,
                                     size_t resp_len, uint32_t timeout_ms);

/**
 * @brief Check whether the system clock has been synchronised from modem
 *        network time (AT+QLTS=2 / NITZ).
 *
 * @return true if settimeofday() has been called at least once with
 *         a valid modem timestamp, false otherwise.
 */
bool lte_upstream_pppos_is_time_synced(void);

// Register the `lte` console command. Called by usb_cli_console.
void lte_upstream_pppos_console_register(void);

#ifdef __cplusplus
}
#endif
