/**
 * @file settings_manager_fields.c
 * @brief Field table -> JSON Schema generator (pure logic, host-testable).
 *
 * Components author their settings as a settings_field_t table (one line per
 * field, see the SETTINGS_* macros); register() calls this to produce the
 * JSON Schema string the validator, defaults collector and
 * GET /settings/<name>/schema already consume. Building through cJSON keeps
 * escaping/formatting correct by construction.
 *
 * Arrays (rev 2.5): SETTINGS_ARRAY rows emit a bounded array-of-objects
 * schema; item fields are scalar rows only (one nesting level) and the
 * bounds mirror the validator's rules (maxItems required, 1..16), so a table
 * the generator accepts always validates.
 */
#include <stdlib.h>
#include <string.h>

#include "settings_manager_private.h"

/* mirror of the validator's SM_ARRAY_MAX_ITEMS (settings_manager_schema.c) */
#define SM_FIELDS_ARRAY_MAX 16

static esp_err_t add_enum(cJSON *prop, const char *csv)
{
    cJSON *arr = cJSON_AddArrayToObject(prop, "enum");

    if (arr == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    const char *s = csv;

    while (*s != '\0')
    {
        const char *comma = strchr(s, ',');
        size_t      len = (comma != NULL) ? (size_t)(comma - s) : strlen(s);

        if (len == 0)
        {
            return ESP_ERR_INVALID_ARG; /* ",," or trailing comma */
        }

        char *item = malloc(len + 1);

        if (item == NULL)
        {
            return ESP_ERR_NO_MEM;
        }

        memcpy(item, s, len);
        item[len] = '\0';
        cJSON_AddItemToArray(arr, cJSON_CreateString(item));
        free(item);

        s += len + ((comma != NULL) ? 1 : 0);
    }

    return cJSON_GetArraySize(arr) > 0 ? ESP_OK : ESP_ERR_INVALID_ARG;
}

static esp_err_t add_object_fields(cJSON *node, const settings_field_t *fields,
                                   size_t count, bool in_items);

static esp_err_t add_array(cJSON *prop, const settings_field_t *f)
{
    if (f->max < 1 || f->max > SM_FIELDS_ARRAY_MAX)
    {
        return ESP_ERR_INVALID_ARG; /* validator: bounded arrays only */
    }

    if (f->min > 0)
    {
        cJSON_AddNumberToObject(prop, "minItems", f->min);
    }

    cJSON_AddNumberToObject(prop, "maxItems", f->max);

    cJSON *items = cJSON_AddObjectToObject(prop, "items");

    if (items == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(items, "type", "object");

    if (f->items != NULL)
    {
        esp_err_t err = add_object_fields(items, f->items, f->item_count,
                                          true);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    /* default: JSON array literal, or [] when absent */
    cJSON *def = (f->def_json != NULL) ? cJSON_Parse(f->def_json)
                                       : cJSON_CreateArray();

    if (def == NULL)
    {
        return (f->def_json != NULL) ? ESP_ERR_INVALID_ARG : ESP_ERR_NO_MEM;
    }

    if (!cJSON_IsArray(def) || cJSON_GetArraySize(def) > f->max)
    {
        cJSON_Delete(def);
        return ESP_ERR_INVALID_ARG; /* default must be an array within bounds */
    }

    cJSON_AddItemToObject(prop, "default", def);
    return ESP_OK;
}

static esp_err_t add_field(cJSON *props, cJSON *required,
                           const settings_field_t *f, bool in_items)
{
    if (f->key == NULL || f->key[0] == '\0' ||
        cJSON_GetObjectItemCaseSensitive(props, f->key) != NULL)
    {
        return ESP_ERR_INVALID_ARG; /* missing or duplicate key */
    }

    if (f->enum_csv != NULL && f->type != SETTINGS_FIELD_STRING)
    {
        return ESP_ERR_INVALID_ARG; /* enums are for strings */
    }

    if (f->type != SETTINGS_FIELD_ARRAY &&
        (f->items != NULL || f->def_json != NULL))
    {
        return ESP_ERR_INVALID_ARG; /* array-only members on a scalar row */
    }

    cJSON *prop = cJSON_AddObjectToObject(props, f->key);

    if (prop == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    switch (f->type)
    {
        case SETTINGS_FIELD_STRING:
            cJSON_AddStringToObject(prop, "type", "string");

            if (f->min > 0)
            {
                cJSON_AddNumberToObject(prop, "minLength", f->min);
            }

            if (f->max > 0)
            {
                cJSON_AddNumberToObject(prop, "maxLength", f->max);
            }

            if (f->enum_csv != NULL)
            {
                esp_err_t err = add_enum(prop, f->enum_csv);

                if (err != ESP_OK)
                {
                    return err;
                }
            }

            cJSON_AddStringToObject(prop, "default",
                                    (f->def_str != NULL) ? f->def_str : "");

            if (f->fmt != NULL)
            {
                cJSON_AddStringToObject(prop, "format", f->fmt);
            }
            break;

        case SETTINGS_FIELD_INT:
            cJSON_AddStringToObject(prop, "type", "integer");

            if (!(f->min == 0 && f->max == 0))
            {
                cJSON_AddNumberToObject(prop, "minimum", f->min);
                cJSON_AddNumberToObject(prop, "maximum", f->max);
            }

            cJSON_AddNumberToObject(prop, "default", f->def_int);
            break;

        case SETTINGS_FIELD_BOOL:
            cJSON_AddStringToObject(prop, "type", "boolean");
            cJSON_AddBoolToObject(prop, "default", f->def_bool);
            break;

        case SETTINGS_FIELD_ARRAY:
        {
            if (in_items)
            {
                return ESP_ERR_INVALID_ARG; /* validator: ONE nesting level */
            }

            cJSON_AddStringToObject(prop, "type", "array");

            esp_err_t err = add_array(prop, f);

            if (err != ESP_OK)
            {
                return err;
            }
            break;
        }

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (f->required)
    {
        cJSON_AddItemToArray(required, cJSON_CreateString(f->key));
    }

    return ESP_OK;
}

/* Build "properties" (+ "required" when any) on @p node from a field table.
   Used for the document root and, with in_items, for an array's items. */
static esp_err_t add_object_fields(cJSON *node, const settings_field_t *fields,
                                   size_t count, bool in_items)
{
    if (fields == NULL || count == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *props = cJSON_AddObjectToObject(node, "properties");
    cJSON *required = cJSON_CreateArray();
    esp_err_t err = ESP_ERR_NO_MEM;

    if (props == NULL || required == NULL)
    {
        goto out;
    }

    for (size_t i = 0; i < count; i++)
    {
        err = add_field(props, required, &fields[i], in_items);

        if (err != ESP_OK)
        {
            goto out;
        }
    }

    if (cJSON_GetArraySize(required) > 0)
    {
        cJSON_AddItemToObject(node, "required", required);
        required = NULL; /* owned by node now */
    }

    err = ESP_OK;

out:
    cJSON_Delete(required);
    return err;
}

esp_err_t sm_schema_from_fields(const settings_field_t *fields, size_t count,
                                char **out_str)
{
    if (fields == NULL || count == 0 || out_str == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_CreateObject();

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(root, "type", "object");

    esp_err_t err = add_object_fields(root, fields, count, false);

    if (err == ESP_OK)
    {
        *out_str = cJSON_PrintUnformatted(root);
        err = (*out_str != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
    }

    cJSON_Delete(root);
    return err;
}
