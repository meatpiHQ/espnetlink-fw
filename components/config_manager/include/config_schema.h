#pragma once

#include <stddef.h>
#include <stdbool.h>

/**
 * Supported value types for schema entries.
 */
typedef enum
{
    CONFIG_TYPE_STRING,
    CONFIG_TYPE_INT,
    CONFIG_TYPE_BOOL,
} config_type_t;

/**
 * One entry in the static configuration schema.
 *
 * The schema array lives in config_schema.c — edit that file to add or remove
 * configuration keys.  The firmware binary carries the schema; config.json on
 * the filesystem only stores runtime values.
 */
typedef struct
{
    const char    *key;          /**< Unique key name (case-sensitive).            */
    config_type_t  type;         /**< Value type.                                  */
    size_t         max_len;      /**< STRING only: max byte length (excl. NUL).    */
    const char    *default_str;  /**< STRING default; NULL is treated as "".       */
    int            default_int;  /**< INT default.                                 */
    bool           default_bool; /**< BOOL default.                                */
} config_schema_entry_t;

/** Defined in config_schema.c — do not declare elsewhere. */
extern const config_schema_entry_t g_config_schema[];
extern const size_t                g_config_schema_count;
