/**
 * @file test_storage_stub.h
 * @brief In-memory sm_storage_* implementation for host tests, plus seeding /
 *        inspection helpers. Lets settings_manager.c (the registry) run on the
 *        linux target with no LittleFS.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Wipe all stored files and counters (call in setUp). */
void sm_stub_reset(void);

/** Seed a stored file directly (bypasses the manager), e.g. old-version data. */
void sm_stub_put(const char *name, uint32_t version, const char *data_json);

/** Corrupt a stored file so its CRC check fails on load. */
void sm_stub_corrupt(const char *name);

/** Number of sm_storage_save() calls since reset (write-dedup assertions). */
int sm_stub_save_count(void);

/** Decode a stored file; returns data object (caller frees) or NULL. */
cJSON *sm_stub_read(const char *name, uint32_t *version_out);

#ifdef __cplusplus
}
#endif
