/**
 * @file main_cli.c
 * @brief Composite console commands owned by the composition root (§6b).
 *
 * Today: the legacy-parity `config` command. Its surface (subcommands,
 * output formats, OK/Error lines) is byte-identical to the retired
 * config_manager's `config` command — the 12 legacy keys now live in
 * per-component settings_manager descriptors and this command is a
 * key-mapped view over them. Sets are reboot-to-apply (§4.2), which
 * matches the legacy semantics (values were read at boot).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_err.h"

#include "cJSON.h"
#include "cmdline_manager.h"
#include "settings_manager.h"

#include "main_compose.h"

#define CFG_VAL_BUF 128

static const cfg_key_t KEYS[] =
{
    { "APN",           "lte_upstream_pppos", "apn",           CFG_KEY_STR,  "internet",   0, false },
    { "APN_USER",      "lte_upstream_pppos", "apn_user",      CFG_KEY_STR,  "",           0, false },
    { "APN_PASS",      "lte_upstream_pppos", "apn_password",  CFG_KEY_STR,  "",           0, false },
    { "APN_PIN",       "lte_upstream_pppos", "apn_pin",       CFG_KEY_STR,  "",           0, false },
    { "DEVICE_NAME",   "system",             "device_name",   CFG_KEY_STR,  "ESPNetLink", 0, false },
    { "LTE_ENABLED",   "lte_upstream_pppos", "enabled",       CFG_KEY_BOOL, NULL,         0, true  },
    { "GPS_ENABLED",   "gps",                "enabled",       CFG_KEY_BOOL, NULL,         0, true  },
    { "NCM_SHARE",     "lte_upstream_pppos", "ncm_share",     CFG_KEY_BOOL, NULL,         0, false },
    { "AGNSS_ENABLED", "gps",                "agnss_enabled", CFG_KEY_BOOL, NULL,         0, true  },
    { "GPS_PASSTHRU",  "gps",                "passthru",      CFG_KEY_BOOL, NULL,         0, false },
    { "GPS_UART0_OUT", "gps",                "uart0_out",     CFG_KEY_BOOL, NULL,         0, false },
    { "AGNSS_CACHE_S", "gps",                "agnss_cache_s", CFG_KEY_INT,  NULL,       120, false },
};

const cfg_key_t *main_cfg_keys(size_t *count)
{
    if (count != NULL)
    {
        *count = sizeof(KEYS) / sizeof(KEYS[0]);
    }

    return KEYS;
}

static const cfg_key_t *find_key(const char *key)
{
    for (size_t i = 0; i < sizeof(KEYS) / sizeof(KEYS[0]); i++)
    {
        if (strcmp(KEYS[i].key, key) == 0)
        {
            return &KEYS[i];
        }
    }

    return NULL;
}

/** Read one mapped key's PENDING value as display text (legacy
 *  config_get_as_str semantics: bool -> "true"/"false", int -> decimal). */
static esp_err_t cfg_get_as_str(const cfg_key_t *k, char *out, size_t out_len)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get(k->comp, &obj);

    if (err != ESP_OK)
    {
        return err;
    }

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, k->field);

    err = ESP_ERR_NOT_FOUND;
    if (k->type == CFG_KEY_STR && cJSON_IsString(item) &&
        item->valuestring != NULL)
    {
        strlcpy(out, item->valuestring, out_len);
        err = ESP_OK;
    }
    else if (k->type == CFG_KEY_INT && cJSON_IsNumber(item))
    {
        snprintf(out, out_len, "%d", (int)item->valuedouble);
        err = ESP_OK;
    }
    else if (k->type == CFG_KEY_BOOL && cJSON_IsBool(item))
    {
        strlcpy(out, cJSON_IsTrue(item) ? "true" : "false", out_len);
        err = ESP_OK;
    }

    cJSON_Delete(obj);
    return err;
}

/** Replace @p k's field in its component object and persist (§4.2). */
static esp_err_t cfg_put_item(const cfg_key_t *k, cJSON *value)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get(k->comp, &obj);

    if (err != ESP_OK)
    {
        cJSON_Delete(value);
        return err;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(obj, k->field);
    cJSON_AddItemToObject(obj, k->field, value);

    err = settings_manager_set(k->comp, obj, NULL, 0, NULL);
    cJSON_Delete(obj);
    return err;
}

/** Legacy config_set_from_str parsing: BOOL accepts 1/true/yes and
 *  0/false/no; INT is strtol base 10; STRING as-is. */
static esp_err_t cfg_set_from_str(const cfg_key_t *k, const char *value_str)
{
    if (k->type == CFG_KEY_STR)
    {
        return cfg_put_item(k, cJSON_CreateString(value_str));
    }

    if (k->type == CFG_KEY_INT)
    {
        char *end = NULL;
        long v = strtol(value_str, &end, 10);

        if (end == NULL || *end != '\0')
        {
            return ESP_ERR_INVALID_ARG;
        }

        return cfg_put_item(k, cJSON_CreateNumber((double)v));
    }

    bool bval;

    if (strcmp(value_str, "1") == 0 || strcmp(value_str, "true") == 0 ||
        strcmp(value_str, "yes") == 0)
    {
        bval = true;
    }
    else if (strcmp(value_str, "0") == 0 || strcmp(value_str, "false") == 0 ||
             strcmp(value_str, "no") == 0)
    {
        bval = false;
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    return cfg_put_item(k, cJSON_CreateBool(bval));
}

static int print_config_list_text(void)
{
    for (size_t i = 0; i < sizeof(KEYS) / sizeof(KEYS[0]); i++)
    {
        const cfg_key_t *e = &KEYS[i];
        char val[CFG_VAL_BUF];

        const char *type_tag =
            (e->type == CFG_KEY_STR) ? "str"  :
            (e->type == CFG_KEY_INT) ? "int"  : "bool";

        esp_err_t ret = cfg_get_as_str(e, val, sizeof(val));
        if (ret == ESP_OK)
        {
            printf("  %-20s [%-4s]  %s\n", e->key, type_tag, val);
        }
        else
        {
            printf("  %-20s [%-4s]  <error: %s>\n",
                   e->key, type_tag, esp_err_to_name(ret));
        }
    }

    return 0;
}

static int print_config_list_json(void)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        printf("Error: %s\n", esp_err_to_name(ESP_ERR_NO_MEM));
        return 1;
    }

    esp_err_t ret = ESP_OK;

    for (size_t i = 0; i < sizeof(KEYS) / sizeof(KEYS[0]) && ret == ESP_OK;
         i++)
    {
        const cfg_key_t *e = &KEYS[i];
        cJSON *obj = NULL;

        ret = settings_manager_get(e->comp, &obj);
        if (ret != ESP_OK)
        {
            break;
        }

        cJSON *item =
            cJSON_Duplicate(cJSON_GetObjectItemCaseSensitive(obj, e->field),
                            1);

        cJSON_Delete(obj);

        if (item == NULL)
        {
            ret = ESP_ERR_NOT_FOUND;
            break;
        }

        cJSON_AddItemToObject(root, e->key, item);
    }

    if (ret == ESP_OK)
    {
        char *json = cJSON_PrintUnformatted(root);
        if (json == NULL)
        {
            ret = ESP_ERR_NO_MEM;
        }
        else
        {
            printf("%s\n", json);
            free(json);
        }
    }

    cJSON_Delete(root);

    if (ret != ESP_OK)
    {
        printf("Error: %s\n", esp_err_to_name(ret));
    }

    return (ret == ESP_OK) ? 0 : 1;
}

static void print_usage(void)
{
    printf("Usage:\n");
    printf("  config list               — print all keys and current values\n");
    printf("  config list -j            — print all keys and current values as JSON\n");
    printf("  config -l [-j]            — short form for list output\n");
    printf("  config get  <KEY>         — print one value\n");
    printf("  config set  <KEY> <VALUE> — set and save a value\n");
    printf("  config reset              — restore all defaults and save\n");
}

static int cmd_config(int argc, char **argv)
{
    if (argc < 2)
    {
        print_usage();
        return 1;
    }

    const char *subcmd = argv[1];

    if (subcmd[0] == '-')
    {
        bool list_requested = false;
        bool json_requested = false;

        for (int i = 1; i < argc; i++)
        {
            if (strcmp(argv[i], "-l") == 0)
            {
                list_requested = true;
            }
            else if (strcmp(argv[i], "-j") == 0)
            {
                json_requested = true;
            }
            else
            {
                print_usage();
                return 1;
            }
        }

        if (!list_requested)
        {
            print_usage();
            return 1;
        }

        return json_requested ? print_config_list_json()
                              : print_config_list_text();
    }

    /* ── list ── */
    if (strcmp(subcmd, "list") == 0)
    {
        if (argc == 2)
        {
            return print_config_list_text();
        }

        if (argc == 3 && strcmp(argv[2], "-j") == 0)
        {
            return print_config_list_json();
        }

        print_usage();
        return 1;
    }

    /* ── reset ── */
    if (strcmp(subcmd, "reset") == 0)
    {
        esp_err_t ret = ESP_OK;

        for (size_t i = 0; i < sizeof(KEYS) / sizeof(KEYS[0]) &&
                           ret == ESP_OK; i++)
        {
            const cfg_key_t *e = &KEYS[i];
            cJSON *value = NULL;

            if (e->type == CFG_KEY_STR)
            {
                value = cJSON_CreateString(e->def_str);
            }
            else if (e->type == CFG_KEY_INT)
            {
                value = cJSON_CreateNumber(e->def_int);
            }
            else
            {
                value = cJSON_CreateBool(e->def_bool);
            }

            ret = cfg_put_item(e, value);
        }

        if (ret == ESP_OK)
        {
            printf("Defaults restored and saved.\n");
        }
        else
        {
            printf("Error: %s\n", esp_err_to_name(ret));
        }
        return (ret == ESP_OK) ? 0 : 1;
    }

    /* ── get / set require a KEY argument ── */
    if (argc < 3)
    {
        print_usage();
        return 1;
    }

    const cfg_key_t *key = find_key(argv[2]);

    /* ── get ── */
    if (strcmp(subcmd, "get") == 0)
    {
        char val[CFG_VAL_BUF];
        esp_err_t ret = (key != NULL) ? cfg_get_as_str(key, val, sizeof(val))
                                      : ESP_ERR_NOT_FOUND;
        if (ret == ESP_OK)
        {
            printf("%s = %s\n", argv[2], val);
        }
        else
        {
            printf("Error (%s): %s\n", argv[2], esp_err_to_name(ret));
        }
        return (ret == ESP_OK) ? 0 : 1;
    }

    /* ── set ── */
    if (strcmp(subcmd, "set") == 0)
    {
        if (argc < 4)
        {
            printf("Usage: config set <KEY> <VALUE>\n");
            return 1;
        }

        esp_err_t ret = (key != NULL) ? cfg_set_from_str(key, argv[3])
                                      : ESP_ERR_NOT_FOUND;
        if (ret != ESP_OK)
        {
            printf("Error: %s\n", esp_err_to_name(ret));
            return 1;
        }

        printf("OK\n");
        return 0;
    }

    print_usage();
    return 1;
}

esp_err_t main_cli_register(void)
{
    static const esp_console_cmd_t CMD_CONFIG =
    {
        .command = "config",
        .help    = "Get/set device config  (config list [-j] | config -l [-j] | get <KEY> | set <KEY> <VAL> | reset)",
        .hint    = NULL,
        .func    = &cmd_config,
    };

    esp_err_t err = cmdline_manager_register(&CMD_CONFIG);

    if (err == ESP_OK)
    {
        err = main_cli_register_system();
    }

    if (err == ESP_OK)
    {
        err = main_cli_register_net();
    }

    if (err == ESP_OK)
    {
        err = main_cli_register_factory();
    }

    return err;
}
