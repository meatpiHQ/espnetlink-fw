#include "config_manager.h"
#include "config_schema.h"

#include "cJSON.h"
#include "esp_console.h"
#include "esp_log.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define CFG_VAL_BUF  128

static int print_config_list_text(void)
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

static int print_config_list_json(void)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        printf("Error: %s\n", esp_err_to_name(ESP_ERR_NO_MEM));
        return 1;
    }

    esp_err_t ret = ESP_OK;

    for (size_t i = 0; i < g_config_schema_count; i++)
    {
        const config_schema_entry_t *e = &g_config_schema[i];

        switch (e->type)
        {
            case CONFIG_TYPE_STRING:
            {
                char val[CFG_VAL_BUF];
                ret = config_get_str(e->key, val, sizeof(val));
                if (ret != ESP_OK)
                {
                    break;
                }
                if (!cJSON_AddStringToObject(root, e->key, val))
                {
                    ret = ESP_ERR_NO_MEM;
                }
                break;
            }

            case CONFIG_TYPE_INT:
            {
                int val = 0;
                ret = config_get_int(e->key, &val);
                if (ret != ESP_OK)
                {
                    break;
                }
                if (!cJSON_AddNumberToObject(root, e->key, val))
                {
                    ret = ESP_ERR_NO_MEM;
                }
                break;
            }

            case CONFIG_TYPE_BOOL:
            {
                bool val = false;
                ret = config_get_bool(e->key, &val);
                if (ret != ESP_OK)
                {
                    break;
                }
                cJSON *item = cJSON_CreateBool(val);
                if (item == NULL)
                {
                    ret = ESP_ERR_NO_MEM;
                    break;
                }
                cJSON_AddItemToObject(root, e->key, item);
                break;
            }
        }

        if (ret != ESP_OK)
        {
            break;
        }
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

        return json_requested ? print_config_list_json() : print_config_list_text();
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
        .help    = "Get/set device config  (config list [-j] | config -l [-j] | get <KEY> | set <KEY> <VAL> | reset)",
        .hint    = NULL,
        .func    = &cmd_config,
    };
    (void)esp_console_cmd_register(&cmd);
}
