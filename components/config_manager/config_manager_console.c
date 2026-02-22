#include "config_manager.h"
#include "config_schema.h"

#include "esp_console.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>

#define CFG_VAL_BUF  128

static void print_usage(void)
{
    printf("Usage:\n");
    printf("  config list               — print all keys and current values\n");
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

    /* ── list ── */
    if (strcmp(subcmd, "list") == 0)
    {
        for (size_t i = 0; i < g_config_schema_count; i++)
        {
            const config_schema_entry_t *e = &g_config_schema[i];
            char val[CFG_VAL_BUF];

            const char *type_tag =
                (e->type == CONFIG_TYPE_STRING) ? "str"  :
                (e->type == CONFIG_TYPE_INT)    ? "int"  : "bool";

            esp_err_t ret = config_get_as_str(e->key, val, sizeof(val));
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

    /* ── reset ── */
    if (strcmp(subcmd, "reset") == 0)
    {
        esp_err_t ret = config_reset_defaults();
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

    const char *key = argv[2];

    /* ── get ── */
    if (strcmp(subcmd, "get") == 0)
    {
        char val[CFG_VAL_BUF];
        esp_err_t ret = config_get_as_str(key, val, sizeof(val));
        if (ret == ESP_OK)
        {
            printf("%s = %s\n", key, val);
        }
        else
        {
            printf("Error (%s): %s\n", key, esp_err_to_name(ret));
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

        esp_err_t ret = config_set_from_str(key, argv[3]);
        if (ret != ESP_OK)
        {
            printf("Error: %s\n", esp_err_to_name(ret));
            return 1;
        }

        ret = config_save();
        if (ret == ESP_OK)
        {
            printf("OK\n");
        }
        else
        {
            printf("Set OK but save failed: %s\n", esp_err_to_name(ret));
        }
        return (ret == ESP_OK) ? 0 : 1;
    }

    print_usage();
    return 1;
}

void config_manager_console_register(void)
{
    const esp_console_cmd_t cmd =
    {
        .command = "config",
        .help    = "Get/set device config  (config list | get <KEY> | set <KEY> <VAL> | reset)",
        .hint    = NULL,
        .func    = &cmd_config,
    };
    (void)esp_console_cmd_register(&cmd);
}
