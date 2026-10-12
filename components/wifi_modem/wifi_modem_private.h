/**
 * @file wifi_modem_private.h
 * @brief Internal contracts of the wifi_modem component: the pure helpers
 *        (wifi_modem_core.c and wifi_modem_migrate.c, host-tested), the
 *        DNS proxy, and the provisioning core. Not part of the public API.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "cJSON.h"

#include "wifi_modem.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- pure helpers (wifi_modem_core.c) ------------------------------------ */

/** "auto" / "ncm" / "console". */
const char *wmc_usb_mode_str(wifi_modem_usb_mode_t mode);

/** Parse "auto" / "ncm" / "console"; false (out untouched) otherwise. */
bool wmc_usb_mode_parse(const char *s, wifi_modem_usb_mode_t *out);

/** Whether the host may switch the USB data lines at runtime in @p mode
 *  (everything but ncm, which locks them on). */
bool wmc_usb_data_switchable(wifi_modem_usb_mode_t mode);

/** Whether the USB device presents the Ethernet function in @p mode:
 *  auto and ncm yes, console no (the CDC console alone). The data lines
 *  are on the DIP route at boot in EVERY mode (2026-10-12). */
bool wmc_usb_ethernet_present(wifi_modem_usb_mode_t mode);

/**
 * Whether a credentials request that arrived on local address @p local_be
 * may be answered: only when it did NOT arrive on a WiFi interface (the AP
 * or STA address, network order, 0 = interface absent). A zero local
 * address (getsockname failed) is refused.
 */
bool wmc_credentials_allowed(uint32_t local_be, uint32_t ap_be, uint32_t sta_be);

/** True when @p password is wifi_manager's schema default (or empty). */
bool wmc_password_is_stock(const char *password);

/**
 * Fill @p out with @p n_chars random characters from an unambiguous
 * alphabet (no 0/O, 1/l/I) using @p rng, NUL-terminated. False when the
 * buffer is too small or n_chars is 0.
 */
bool wmc_generate_password(char *out, size_t out_len, size_t n_chars,
                           uint32_t (*rng)(void));

typedef enum
{
    WMC_PROVISION_NONE = 0,  /**< already provisioned: nothing to do     */
    WMC_PROVISION_MARK,      /**< custom password: only mark provisioned  */
    WMC_PROVISION_GENERATE,  /**< stock password: generate + persist      */
} wmc_provision_action_t;

/** The provisioning decision, as a pure function of the two inputs. */
wmc_provision_action_t wmc_provision_decide(bool provisioned,
                                            const char *ap_password);

/** The resolver the proxy forwards to: the learned one, else 8.8.8.8.
 *  Both values network order. */
uint32_t wmc_pick_upstream_dns(uint32_t learned_be);

/** Dotted quad of a network-order IPv4; false when @p out is too small. */
bool wmc_format_ipv4(uint32_t addr_be, char *out, size_t out_len);

/* ---- settings migration (wifi_modem_migrate.c, pure) ----------------------- */

/**
 * Migrate a stored "wifi_modem" object from @p from_version to the current
 * schema, in place. v1 (2026-08-23 to 2026-10-12) carried `usb_mode` with
 * the value `power_only` and the device-state key `boot_cut`: `power_only`
 * becomes `console` (the same charger-powered, AP-only use with a way back
 * in), `boot_cut` is dropped (nothing about a cut persists any more). Any
 * other stored value is left for the schema validation that follows.
 * ESP_ERR_INVALID_ARG when @p settings is not an object.
 */
esp_err_t wifi_modem_settings_migrate(uint32_t from_version, cJSON *settings);

/* ---- settings (wifi_modem_settings.c) ------------------------------------ */

/** Register the "wifi_modem" descriptor (called from wifi_modem_init). */
esp_err_t wifi_modem_settings_register(void);

/* ---- provisioning core (wifi_modem_provision.c) -------------------------- */

/** wifi_modem_provision() with an injectable RNG (host tests). */
esp_err_t wifi_modem_provision_with(uint32_t (*rng)(void),
                                    bool *reboot_required);

/* ---- DNS proxy (wifi_modem_dns.c) ---------------------------------------- */

/** UDP/53 forwarder for AP clients, bound to the AP address @p local_be
 *  (network order) so it never answers for the USB link; relays to the
 *  upstream resolver. Idempotent start; stop joins the task. */
esp_err_t wifi_modem_dns_start(uint32_t local_be);
void wifi_modem_dns_stop(void);
void wifi_modem_dns_set_upstream(uint32_t addr_be);

#ifdef __cplusplus
}
#endif
