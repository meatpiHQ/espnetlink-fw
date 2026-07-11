/**
 * @file settings_manager.h
 * @brief WiCAN Settings Manager — public API.
 *
 * The Settings Manager owns settings persistence and validation for the whole
 * firmware. Components do not touch the filesystem: they register a descriptor
 * (schema + hooks). Settings are REBOOT-TO-APPLY (Coding Standard rev 2 §4.2):
 * set() validates and persists only; on_apply runs once per component at boot,
 * single-threaded, before any component's _start(). There is no live-update
 * path and components need no reload code.
 *
 * See components/settings_manager/README.md and ARCHITECTURE.md.
 *
 * Lifecycle (called by main, in dependency order):
 *   settings_manager_init();   // mount FS, prepare registry. No on_apply yet.
 *   <comp>_init();             // each component registers its descriptor
 *   settings_manager_start();  // load -> migrate -> validate -> on_apply, per §4.3
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- field-table schema authoring ----------------------------------------- */

/**
 * Components author their schema as a C table of settings_field_t; the manager
 * generates the equivalent JSON Schema once at registration. JSON Schema
 * remains the single source of truth on the wire (validation +
 * GET /settings/<name>/schema) — the table is the authoring format. Since
 * rev 2.5 tables express bounded arrays of objects too, so every component
 * schema is a field table; raw .schema strings remain only as the manager's
 * internal wire format (and for tests).
 */
typedef enum
{
    SETTINGS_FIELD_STRING = 0,
    SETTINGS_FIELD_INT,
    SETTINGS_FIELD_BOOL,
    SETTINGS_FIELD_ARRAY,           /**< bounded array of objects (rev 2.5)      */
} settings_field_type_t;

typedef struct settings_field settings_field_t;

struct settings_field
{
    const char           *key;
    settings_field_type_t type;
    int32_t               min;      /**< STRING: minLength; INT: minimum;        */
                                    /*   ARRAY: minItems (0 => none).            */
    int32_t               max;      /**< STRING: maxLength; INT: maximum;        */
                                    /*   ARRAY: maxItems — REQUIRED, 1..16       */
                                    /*   (validator: bounded arrays only).       */
                                    /*   min == 0 && max == 0 => no bounds       */
                                    /*   (scalars only).                         */
    const char           *enum_csv; /**< STRING only: "a,b,c" allowed values.    */
    const char           *def_str;  /**< STRING default (NULL => "").            */
    int32_t               def_int;  /**< INT default.                            */
    bool                  def_bool; /**< BOOL default.                           */
    bool                  required; /**< Adds the key to the "required" list.    */
    const char           *fmt;      /**< WiCAN extension, e.g. "file". Or NULL.  */

    /* ARRAY only. Item objects' fields: scalar rows only — ONE nesting level,
       matching the validator (settings_manager_schema.c). NULL items =>
       free-form objects (shape enforced by on_validate instead). def_json is
       the array's default as a JSON array literal; NULL => [] default.        */
    const settings_field_t *items;
    size_t                  item_count;
    const char             *def_json;
};

/* Row helpers — keep tables one line per field:
 *   SETTINGS_STR("sta_ssid", 32, "")
 *   SETTINGS_STR_ENUM("mode", "off,sta,ap,apsta", "apsta")
 *   SETTINGS_INT("ap_channel", 1, 13, 6)
 *   SETTINGS_BOOL("ap_auto_disable", false)
 *   SETTINGS_ARRAY("servers", 4, SERVER_ITEM_FIELDS, "[{\"name\":\"obd0\"}]")
 * _REQ variants mark the key required — meaningful inside array items (a PUT
 * of the top-level object always has every key filled from defaults first).  */
#define SETTINGS_STR(k, maxlen, def) \
    { (k), SETTINGS_FIELD_STRING, 0, (maxlen), NULL, (def), 0, false, false, NULL, NULL, 0, NULL }
#define SETTINGS_STR_LEN(k, minlen, maxlen, def) \
    { (k), SETTINGS_FIELD_STRING, (minlen), (maxlen), NULL, (def), 0, false, false, NULL, NULL, 0, NULL }
#define SETTINGS_STR_ENUM(k, csv, def) \
    { (k), SETTINGS_FIELD_STRING, 0, 0, (csv), (def), 0, false, false, NULL, NULL, 0, NULL }
#define SETTINGS_STR_FILE(k, maxlen, def) \
    { (k), SETTINGS_FIELD_STRING, 0, (maxlen), NULL, (def), 0, false, false, "file", NULL, 0, NULL }
#define SETTINGS_INT(k, mn, mx, def) \
    { (k), SETTINGS_FIELD_INT, (mn), (mx), NULL, NULL, (def), false, false, NULL, NULL, 0, NULL }
#define SETTINGS_BOOL(k, def) \
    { (k), SETTINGS_FIELD_BOOL, 0, 0, NULL, NULL, 0, (def), false, NULL, NULL, 0, NULL }

/* Required-key variants (for array item tables). */
#define SETTINGS_STR_REQ(k, minlen, maxlen, def) \
    { (k), SETTINGS_FIELD_STRING, (minlen), (maxlen), NULL, (def), 0, false, true, NULL, NULL, 0, NULL }
#define SETTINGS_STR_ENUM_REQ(k, csv, def) \
    { (k), SETTINGS_FIELD_STRING, 0, 0, (csv), (def), 0, false, true, NULL, NULL, 0, NULL }
#define SETTINGS_INT_REQ(k, mn, mx, def) \
    { (k), SETTINGS_FIELD_INT, (mn), (mx), NULL, NULL, (def), false, true, NULL, NULL, 0, NULL }

/* Stringize a JSON literal so array defaults are written as plain JSON —
 * no \" escaping (the preprocessor inserts the escapes; nested string
 * literals inside the JSON keep exactly their source spelling):
 *   SETTINGS_JSON([{"name":"obd0","proto":"tcp","port":35000}])
 * Inter-token whitespace collapses to single spaces, so it may span lines.
 * GOTCHA: each JSON string must stay ONE C token — adjacent-literal
 * concatenation ("a" "b") happens AFTER stringization and would leave two
 * separate quoted fragments in the JSON text (a parse error). */
#define SETTINGS_JSON(...) #__VA_ARGS__

/* Bounded array of objects. item_fields is a settings_field_t[] of scalar
 * rows (its per-row defaults serve as UI "add row" prefill hints); def is a
 * JSON array literal (use SETTINGS_JSON) or NULL for []. SETTINGS_ARRAY_ANY
 * validates items as free-form objects — pair it with an on_validate that
 * enforces the shape. */
#define SETTINGS_ARRAY(k, max_items, item_fields, def) \
    { (k), SETTINGS_FIELD_ARRAY, 0, (max_items), NULL, NULL, 0, false, false, NULL, \
      (item_fields), sizeof(item_fields) / sizeof((item_fields)[0]), (def) }
#define SETTINGS_ARRAY_ANY(k, max_items, def) \
    { (k), SETTINGS_FIELD_ARRAY, 0, (max_items), NULL, NULL, 0, false, false, NULL, \
      NULL, 0, (def) }

/**
 * @brief Descriptor a component registers with the Settings Manager.
 *
 * All pointers must remain valid for the lifetime of the program (point them at
 * static/flash data). The manager does not copy @c schema or @c defaults_json.
 */
typedef struct
{
    const char *name;          /**< Component id. Settings key + on-disk filename. */
    uint32_t    version;       /**< Schema version, for migrations.                */
    const char *schema;        /**< Raw JSON Schema string. Components use @c       */
                               /*   fields instead (standard §4.1, rev 2.5); this   */
                               /*   remains for the manager's own tests. NULL when  */
                               /*   @c fields is set.                               */

    const settings_field_t *fields; /**< Field-table schema (the component          */
    size_t field_count;             /*   authoring format). Exactly one of          */
                                    /*   schema/fields must be set.                 */

    const char *defaults_json; /**< OPTIONAL whole-object override of the schema-   */
                               /*   derived defaults (JSON object string). When set  */
                               /*   it replaces them wholesale. NULL for most       */
                               /*   components. Must parse as a JSON object.        */

    /**
     * Called ONCE, at boot, in settings_manager_start()'s context, before any
     * component's _start(). Never called at runtime — settings changes persist
     * only and take effect after reboot (Coding Standard §4.2). If this fails,
     * the manager persists defaults and calls it once more with them (§4.3).
     */
    esp_err_t (*on_apply)(const cJSON *settings);

    /** Optional: reject bad input before commit (cross-field rules). May be NULL. */
    esp_err_t (*on_validate)(const cJSON *settings, char *err, size_t err_len);

    /**
     * Optional: migrate stored settings from an older schema version. Called once
     * at boot when the stored version < @c version, with the stored version;
     * must handle ANY from_version the component has ever shipped. Mutates
     * @p settings in place. The result is re-validated against the schema and
     * persisted. Absent or failing => defaults are used (and an error logged).
     */
    esp_err_t (*on_migrate)(uint32_t from_version, cJSON *settings);
} settings_descriptor_t;

/**
 * @brief Mount the settings filesystem and prepare the registry.
 * @note  Does not load or apply anything yet. No bus side effects.
 */
esp_err_t settings_manager_init(void);

/**
 * @brief Load, migrate, validate, and apply settings for every registered
 *        component (Coding Standard §4.3). Single-threaded; call before any
 *        component's _start().
 *
 * Per component: read + CRC-verify the file; if the stored version is older,
 * run on_migrate and re-validate; validate (schema + on_validate); on_apply.
 * Any failure falls back to defaults (persisted, logged). If on_apply rejects
 * even the defaults the component is left unconfigured and marked degraded —
 * its _start() must then refuse with ESP_ERR_INVALID_STATE (§3). One broken
 * component never bricks boot; there are no retry loops or reboots here.
 */
esp_err_t settings_manager_start(void);

/**
 * @brief Release resources and unmount the filesystem.
 */
esp_err_t settings_manager_stop(void);

/**
 * @brief FACTORY RESET: delete every stored settings file. The running
 *        configuration is untouched — the caller MUST reboot (via
 *        restart_tracker, reason FACTORY_RESET) so the next boot's load
 *        pass finds nothing and applies pure factory defaults.
 *
 * Scope: the settings partition only. User data on /data (certificate
 * sets, uploaded files) and NVS (WiFi PHY cal) are NOT touched.
 */
esp_err_t settings_manager_factory_reset(void);

/**
 * @brief Register a component descriptor. Call between init and start.
 *
 * The schema must parse as JSON and the name must be a valid identifier
 * ([a-z0-9_], at most 32 chars) since it becomes the on-disk filename.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG (bad descriptor / unparseable schema /
 *         invalid name), ESP_ERR_INVALID_STATE (already started or duplicate),
 *         or ESP_ERR_NO_MEM (registry full).
 */
esp_err_t settings_manager_register(const settings_descriptor_t *desc);

/**
 * @brief Get a copy of a component's current settings.
 * @param[out] out  Receives a newly allocated cJSON object. Caller frees with
 *                  cJSON_Delete().
 */
esp_err_t settings_manager_get(const char *name, cJSON **out);

/**
 * @brief Validate and persist a settings change. NEVER applies it.
 *
 * Reboot-to-apply (Coding Standard §4.2): @p in is the COMPLETE new settings
 * object (full replace — the transport's PUT semantics). Keys still missing
 * after that are filled from the schema defaults, the result is validated
 * against the schema and on_validate, then persisted atomically. on_apply is
 * NOT called; the change takes effect at the next boot.
 *
 * If the result is byte-identical to what is already persisted, nothing is
 * written (flash-wear dedup) and @p changed reports false — the transport uses
 * this to skip the post-submit reboot.
 *
 * @param err      Optional buffer receiving a human-readable failure reason,
 *                 private to this call (safe under concurrent sets). May be NULL.
 * @param err_len  Size of @p err.
 * @param changed  Optional. true if a write happened; false on a no-op set.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on validation failure.
 * @note API change (rev 2): no longer merges over current settings, no longer
 *       calls on_apply, and gained @p changed.
 */
esp_err_t settings_manager_set(const char *name, const cJSON *in,
                               char *err, size_t err_len, bool *changed);

/**
 * @brief True if the boot fallback fired for @p name (stored settings could not
 *        be migrated/validated/applied and defaults were used — Coding Standard
 *        §4.3 steps 4–5). Transports add "degraded":true to GET responses from
 *        this; the manager never injects synthetic keys into settings objects.
 *        Unknown names return false.
 */
bool settings_manager_is_degraded(const char *name);

/**
 * @brief True when @p name's PENDING (persisted) object differs from what
 *        on_apply actually ran with at boot — i.e. a reboot is needed for the
 *        saved settings to take effect. Exact: saving values back to the
 *        boot-applied state clears it. Unknown/unstarted names return false.
 *        Transports surface it as "pending_reboot" (UI: "restart to apply").
 */
bool settings_manager_is_pending_reboot(const char *name);

/**
 * @brief Get a copy of a component's schema as parsed JSON (for UIs/tools to
 *        self-describe).
 * @param[out] out  Receives a newly allocated cJSON object. Caller frees with
 *                  cJSON_Delete().
 * @note API change: previously returned a borrowed pointer, which could dangle
 *       across settings_manager_stop(). Now symmetric with settings_manager_get.
 */
esp_err_t settings_manager_get_schema(const char *name, cJSON **out);

/**
 * @brief List registered components as an array of {"name","version"} objects.
 * @param[out] out  Receives a newly allocated cJSON array. Caller frees.
 */
esp_err_t settings_manager_list(cJSON **out);

/**
 * @brief Human-readable description of the last validation/IO failure.
 * @return A pointer to an internal buffer; valid until the next manager call.
 * @warning Shared across callers — under concurrent access (e.g. httpd workers)
 *          prefer the err out-param on settings_manager_set().
 */
const char *settings_manager_last_error(void);

/**
 * @brief Export every registered component's settings for backup/transfer.
 * @param[out] out  Newly allocated `{"<name>":{"version":N,"data":{...}}, ...}`
 *                  — the PENDING values (get() semantics) plus each schema
 *                  version, so an old backup restores through migrations.
 *                  Caller frees with cJSON_Delete().
 * @warning Values are exported VERBATIM, password fields included: a backup
 *          that loses secrets cannot transfer a configuration to another
 *          device. Redaction (if wanted) is the transport's decision.
 */
esp_err_t settings_manager_export(cJSON **out);

/**
 * @brief Restore one component from a backup taken at schema @p version.
 *
 * Pipeline: migrate via the component's on_migrate when @p version is older
 * than the registered schema (exactly like the boot pass would) -> fill
 * omitted keys from schema defaults -> validate -> persist (set() semantics:
 * reboot-to-apply, write-dedup). A @p version NEWER than the firmware's is
 * rejected with ESP_ERR_INVALID_VERSION ("update the firmware first").
 *
 * @param dry_run  Stop after validation, persist nothing — lets a transport
 *                 check EVERY component first and make a restore
 *                 all-or-nothing.
 * @param err/err_len  Failure reason for the UI (may be NULL/0).
 * @param[out] changed  True if something was actually persisted (false on
 *                      dry runs and identical data). May be NULL.
 */
esp_err_t settings_manager_restore(const char *name, uint32_t version,
                                   const cJSON *in, bool dry_run,
                                   char *err, size_t err_len, bool *changed);

#ifdef __cplusplus
}
#endif
