/**
 * @file settings_manager_schema.c
 * @brief WiCAN settings schema validator. A clean subset of JSON Schema plus the
 *        format:"file" extension. Pure logic — host-testable on the linux target.
 *
 * Supported keywords:
 *   type        string | integer | number | boolean | object | array
 *   minimum     inclusive lower bound (integer/number)
 *   maximum     inclusive upper bound (integer/number)
 *   minLength   string min length
 *   maxLength   string max length
 *   enum        array of allowed values (any type)
 *   default     value used to fill the key when it is absent (collected by
 *               sm_schema_collect_defaults; not a validation constraint)
 *   required    array of keys that must be present
 *   format      "file" -> value names a file on the FS (existence is checked)
 *   items       (arrays) schema applied to every element — a scalar schema,
 *               or type:"object" with its own properties/required (one level
 *               of nesting; arrays inside array items are not supported)
 *   maxItems    (arrays) REQUIRED bound, <= 16 — bounded arrays only, so the
 *               transports and the on-disk envelope stay small
 *   minItems    (arrays) optional lower bound
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "settings_manager_private.h"

/* Write a formatted reason into the (optional) error buffer and fail. */
static esp_err_t fail(char *err, size_t err_len, const char *fmt, ...)
{
    if (err != NULL && err_len > 0)
    {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(err, err_len, fmt, ap);
        va_end(ap);
    }

    return ESP_ERR_INVALID_ARG;
}

static bool number_is_integer(const cJSON *v)
{
    double d = v->valuedouble;
    return (floor(d) == d) && !isinf(d);
}

/* enum: the value must deep-equal one of the listed entries. */
static esp_err_t check_enum(const char *key, const cJSON *enum_arr,
                            const cJSON *value, char *err, size_t err_len)
{
    const cJSON *opt = NULL;

    cJSON_ArrayForEach(opt, enum_arr)
    {
        if (cJSON_Compare(opt, value, true))
        {
            return ESP_OK;
        }
    }

    return fail(err, err_len, "%s: value not one of the allowed options", key);
}

#define SM_ARRAY_MAX_ITEMS 16 /* hard ceiling for any bounded array */

static esp_err_t check_object_node(const cJSON *schema_node, const cJSON *data,
                                   bool (*file_exists)(const char *name),
                                   char *err, size_t err_len);

/* Validate a single property value against its property schema. */
static esp_err_t check_property(const char *key, const cJSON *prop_schema,
                                const cJSON *value,
                                bool (*file_exists)(const char *name),
                                char *err, size_t err_len)
{
    const cJSON *type_item = cJSON_GetObjectItemCaseSensitive(prop_schema, "type");
    const char  *type = cJSON_IsString(type_item) ? type_item->valuestring : NULL;

    const cJSON *enum_arr = cJSON_GetObjectItemCaseSensitive(prop_schema, "enum");

    if (cJSON_IsArray(enum_arr))
    {
        esp_err_t e = check_enum(key, enum_arr, value, err, err_len);

        if (e != ESP_OK)
        {
            return e;
        }
    }

    if (type == NULL)
    {
        return ESP_OK;   /* enum-only / untyped property already handled above */
    }

    if (strcmp(type, "string") == 0)
    {
        if (!cJSON_IsString(value))
        {
            return fail(err, err_len, "%s: expected string", key);
        }

        size_t len = strlen(value->valuestring);

        const cJSON *min_l = cJSON_GetObjectItemCaseSensitive(prop_schema, "minLength");
        const cJSON *max_l = cJSON_GetObjectItemCaseSensitive(prop_schema, "maxLength");

        if (cJSON_IsNumber(min_l) && len < (size_t)min_l->valuedouble)
        {
            return fail(err, err_len, "%s: shorter than minLength %d",
                        key, (int)min_l->valuedouble);
        }

        if (cJSON_IsNumber(max_l) && len > (size_t)max_l->valuedouble)
        {
            return fail(err, err_len, "%s: longer than maxLength %d",
                        key, (int)max_l->valuedouble);
        }

        const cJSON *fmt = cJSON_GetObjectItemCaseSensitive(prop_schema, "format");

        if (cJSON_IsString(fmt) && strcmp(fmt->valuestring, "file") == 0)
        {
            if (file_exists != NULL && !file_exists(value->valuestring))
            {
                return fail(err, err_len, "%s: file '%s' not found",
                            key, value->valuestring);
            }
        }

        return ESP_OK;
    }

    if (strcmp(type, "integer") == 0 || strcmp(type, "number") == 0)
    {
        if (!cJSON_IsNumber(value))
        {
            return fail(err, err_len, "%s: expected %s", key, type);
        }

        if (strcmp(type, "integer") == 0 && !number_is_integer(value))
        {
            return fail(err, err_len, "%s: expected integer", key);
        }

        const cJSON *min = cJSON_GetObjectItemCaseSensitive(prop_schema, "minimum");
        const cJSON *max = cJSON_GetObjectItemCaseSensitive(prop_schema, "maximum");

        if (cJSON_IsNumber(min) && value->valuedouble < min->valuedouble)
        {
            return fail(err, err_len, "%s: below minimum %g", key, min->valuedouble);
        }

        if (cJSON_IsNumber(max) && value->valuedouble > max->valuedouble)
        {
            return fail(err, err_len, "%s: above maximum %g", key, max->valuedouble);
        }

        return ESP_OK;
    }

    if (strcmp(type, "boolean") == 0)
    {
        if (!cJSON_IsBool(value))
        {
            return fail(err, err_len, "%s: expected boolean", key);
        }

        return ESP_OK;
    }

    if (strcmp(type, "object") == 0)
    {
        if (!cJSON_IsObject(value))
        {
            return fail(err, err_len, "%s: expected object", key);
        }

        return ESP_OK;
    }

    if (strcmp(type, "array") == 0)
    {
        if (!cJSON_IsArray(value))
        {
            return fail(err, err_len, "%s: expected array", key);
        }

        /* bounded arrays ONLY: a schema without maxItems is an authoring
           bug — fail loudly rather than accept unbounded input */
        const cJSON *max_i = cJSON_GetObjectItemCaseSensitive(prop_schema,
                                                              "maxItems");
        const cJSON *min_i = cJSON_GetObjectItemCaseSensitive(prop_schema,
                                                              "minItems");

        if (!cJSON_IsNumber(max_i) ||
            max_i->valuedouble > SM_ARRAY_MAX_ITEMS || max_i->valuedouble < 1)
        {
            return fail(err, err_len,
                        "%s: schema error - array needs maxItems 1..%d",
                        key, SM_ARRAY_MAX_ITEMS);
        }

        int count = cJSON_GetArraySize(value);

        if (count > (int)max_i->valuedouble)
        {
            return fail(err, err_len, "%s: more than maxItems %d entries",
                        key, (int)max_i->valuedouble);
        }

        if (cJSON_IsNumber(min_i) && count < (int)min_i->valuedouble)
        {
            return fail(err, err_len, "%s: fewer than minItems %d entries",
                        key, (int)min_i->valuedouble);
        }

        const cJSON *items = cJSON_GetObjectItemCaseSensitive(prop_schema,
                                                              "items");

        if (!cJSON_IsObject(items))
        {
            return ESP_OK; /* untyped elements -> allowed */
        }

        const cJSON *it_type = cJSON_GetObjectItemCaseSensitive(items, "type");
        bool item_is_object = cJSON_IsString(it_type) &&
                              strcmp(it_type->valuestring, "object") == 0;
        const cJSON *element = NULL;
        int idx = 0;

        cJSON_ArrayForEach(element, value)
        {
            char ekey[48];

            snprintf(ekey, sizeof(ekey), "%s[%d]", key, idx);

            if (item_is_object)
            {
                if (!cJSON_IsObject(element))
                {
                    return fail(err, err_len, "%s: expected object", ekey);
                }

                char inner[96] = "";
                esp_err_t e = check_object_node(items, element, file_exists,
                                                inner, sizeof(inner));

                if (e != ESP_OK)
                {
                    return fail(err, err_len, "%s: %s", ekey, inner);
                }
            }
            else
            {
                esp_err_t e = check_property(ekey, items, element,
                                             file_exists, err, err_len);

                if (e != ESP_OK)
                {
                    return e;
                }
            }

            idx++;
        }

        return ESP_OK;
    }

    return fail(err, err_len, "%s: unknown schema type '%s'", key, type);
}

/* required + per-property validation of one object against a parsed schema
   node (the document root, or an array's items schema — one nesting level). */
static esp_err_t check_object_node(const cJSON *schema_node, const cJSON *data,
                                   bool (*file_exists)(const char *name),
                                   char *err, size_t err_len)
{
    const cJSON *required = cJSON_GetObjectItemCaseSensitive(schema_node,
                                                             "required");

    if (cJSON_IsArray(required))
    {
        const cJSON *req = NULL;

        cJSON_ArrayForEach(req, required)
        {
            if (!cJSON_IsString(req))
            {
                continue;
            }

            if (cJSON_GetObjectItemCaseSensitive(data,
                                                 req->valuestring) == NULL)
            {
                return fail(err, err_len, "missing required field '%s'",
                            req->valuestring);
            }
        }
    }

    /* unknown keys are permitted (additionalProperties defaults to true) */
    const cJSON *properties = cJSON_GetObjectItemCaseSensitive(schema_node,
                                                               "properties");

    if (cJSON_IsObject(properties))
    {
        const cJSON *value = NULL;

        cJSON_ArrayForEach(value, data)
        {
            const cJSON *prop_schema =
                cJSON_GetObjectItemCaseSensitive(properties, value->string);

            if (prop_schema == NULL)
            {
                continue; /* not described by schema -> allowed */
            }

            esp_err_t e = check_property(value->string, prop_schema, value,
                                         file_exists, err, err_len);

            if (e != ESP_OK)
            {
                return e;
            }
        }
    }

    return ESP_OK;
}

esp_err_t sm_schema_validate(const char *schema_json, const cJSON *data,
                             bool (*file_exists)(const char *name),
                             char *err, size_t err_len)
{
    if (schema_json == NULL || data == NULL)
    {
        return fail(err, err_len, "internal: null schema or data");
    }

    if (!cJSON_IsObject(data))
    {
        return fail(err, err_len, "settings must be a JSON object");
    }

    cJSON *schema = cJSON_Parse(schema_json);

    if (schema == NULL)
    {
        return fail(err, err_len, "internal: schema is not valid JSON");
    }

    esp_err_t result = check_object_node(schema, data, file_exists,
                                         err, err_len);

    cJSON_Delete(schema);
    return result;
}

esp_err_t sm_schema_collect_defaults(const char *schema_json, cJSON **out)
{
    if (schema_json == NULL || out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *result = cJSON_CreateObject();

    if (result == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    cJSON *schema = cJSON_Parse(schema_json);

    if (schema != NULL)
    {
        const cJSON *properties =
            cJSON_GetObjectItemCaseSensitive(schema, "properties");

        if (cJSON_IsObject(properties))
        {
            const cJSON *prop = NULL;

            cJSON_ArrayForEach(prop, properties)
            {
                const cJSON *def = cJSON_GetObjectItemCaseSensitive(prop, "default");

                if (def != NULL)
                {
                    cJSON_AddItemToObject(result, prop->string,
                                          cJSON_Duplicate(def, true));
                }
            }
        }

        cJSON_Delete(schema);
    }

    *out = result;
    return ESP_OK;
}
