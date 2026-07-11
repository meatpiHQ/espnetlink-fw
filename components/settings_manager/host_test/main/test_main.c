/**
 * @file test_main.c
 * @brief Host test entry point. Runs the schema + codec unit suites on the
 *        linux target (no hardware required).
 */
#include "unity.h"

/* schema */
void test_valid_object_accepted(void);
void test_missing_required_rejected(void);
void test_wrong_type_rejected(void);
void test_integer_out_of_range_rejected(void);
void test_non_integer_for_integer_rejected(void);
void test_number_in_range_accepted(void);
void test_enum_member_accepted(void);
void test_enum_non_member_rejected(void);
void test_string_too_long_rejected(void);
void test_boolean_type_enforced(void);
void test_file_ref_present_accepted(void);
void test_file_ref_missing_rejected(void);
void test_unknown_key_allowed(void);
void test_collect_defaults(void);

/* bounded arrays (2026-07-03, for socket_manager/bridge_manager) */
void test_array_of_objects_accepted(void);
void test_array_over_max_items_rejected(void);
void test_array_item_bad_field_rejected_with_index(void);
void test_array_item_missing_required_rejected(void);
void test_array_item_enum_enforced(void);
void test_scalar_array_items_checked(void);
void test_array_without_max_items_is_schema_error(void);
void test_non_array_for_array_rejected(void);
void test_array_default_collected(void);

/* registry (rev 2 contract) */
void test_set_never_applies(void);
void test_changed_flag_and_dedup(void);
void test_defaults_json_whole_object_override(void);
void test_schema_defaults_primary(void);
void test_migrate_happy_path(void);
void test_migrate_failure_falls_back_to_defaults(void);
void test_migrate_invalid_result_falls_back(void);
void test_old_version_without_migrate_falls_back(void);
void test_apply_failure_retries_with_defaults(void);
void test_defaults_also_fail_leaves_unconfigured(void);
void test_register_rejects_bad_defaults_json(void);
void test_register_after_start_rejected(void);
void test_set_is_full_replace_not_merge(void);

/* backup / restore */
void test_export_shape_and_versions(void);
void test_restore_same_version_persists(void);
void test_restore_migrates_older_backup(void);
void test_restore_newer_backup_rejected(void);
void test_restore_dry_run_validates_but_persists_nothing(void);
void test_restore_unknown_component_not_found(void);
void test_restore_identical_reports_unchanged(void);
void test_pending_reboot_lifecycle(void);

/* codec */
void test_crc32_known_vector(void);
void test_encode_decode_roundtrip(void);
void test_decode_detects_tamper(void);
void test_decode_rejects_malformed(void);

/* field-table schema generator */
void test_fields_generated_schema_parses_with_expected_keywords(void);
void test_fields_roundtrip_validator_accepts_and_rejects(void);
void test_fields_defaults_collected(void);
void test_fields_rejects_duplicate_and_empty_keys(void);
void test_fields_required_emitted(void);
void test_fields_array_schema_shape(void);
void test_fields_array_roundtrip_validator(void);
void test_fields_array_defaults_collected(void);
void test_fields_settings_json_stringize(void);
void test_fields_array_rejects_malformed_tables(void);

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_valid_object_accepted);
    RUN_TEST(test_missing_required_rejected);
    RUN_TEST(test_wrong_type_rejected);
    RUN_TEST(test_integer_out_of_range_rejected);
    RUN_TEST(test_non_integer_for_integer_rejected);
    RUN_TEST(test_number_in_range_accepted);
    RUN_TEST(test_enum_member_accepted);
    RUN_TEST(test_enum_non_member_rejected);
    RUN_TEST(test_string_too_long_rejected);
    RUN_TEST(test_boolean_type_enforced);
    RUN_TEST(test_file_ref_present_accepted);
    RUN_TEST(test_file_ref_missing_rejected);
    RUN_TEST(test_unknown_key_allowed);
    RUN_TEST(test_collect_defaults);

    RUN_TEST(test_array_of_objects_accepted);
    RUN_TEST(test_array_over_max_items_rejected);
    RUN_TEST(test_array_item_bad_field_rejected_with_index);
    RUN_TEST(test_array_item_missing_required_rejected);
    RUN_TEST(test_array_item_enum_enforced);
    RUN_TEST(test_scalar_array_items_checked);
    RUN_TEST(test_array_without_max_items_is_schema_error);
    RUN_TEST(test_non_array_for_array_rejected);
    RUN_TEST(test_array_default_collected);

    RUN_TEST(test_set_never_applies);
    RUN_TEST(test_changed_flag_and_dedup);
    RUN_TEST(test_defaults_json_whole_object_override);
    RUN_TEST(test_schema_defaults_primary);
    RUN_TEST(test_migrate_happy_path);
    RUN_TEST(test_migrate_failure_falls_back_to_defaults);
    RUN_TEST(test_migrate_invalid_result_falls_back);
    RUN_TEST(test_old_version_without_migrate_falls_back);
    RUN_TEST(test_apply_failure_retries_with_defaults);
    RUN_TEST(test_defaults_also_fail_leaves_unconfigured);
    RUN_TEST(test_register_rejects_bad_defaults_json);
    RUN_TEST(test_register_after_start_rejected);
    RUN_TEST(test_set_is_full_replace_not_merge);

    RUN_TEST(test_export_shape_and_versions);
    RUN_TEST(test_restore_same_version_persists);
    RUN_TEST(test_restore_migrates_older_backup);
    RUN_TEST(test_restore_newer_backup_rejected);
    RUN_TEST(test_restore_dry_run_validates_but_persists_nothing);
    RUN_TEST(test_restore_unknown_component_not_found);
    RUN_TEST(test_restore_identical_reports_unchanged);
    RUN_TEST(test_pending_reboot_lifecycle);

    RUN_TEST(test_crc32_known_vector);
    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_decode_detects_tamper);
    RUN_TEST(test_decode_rejects_malformed);

    RUN_TEST(test_fields_generated_schema_parses_with_expected_keywords);
    RUN_TEST(test_fields_roundtrip_validator_accepts_and_rejects);
    RUN_TEST(test_fields_defaults_collected);
    RUN_TEST(test_fields_rejects_duplicate_and_empty_keys);
    RUN_TEST(test_fields_required_emitted);
    RUN_TEST(test_fields_array_schema_shape);
    RUN_TEST(test_fields_array_roundtrip_validator);
    RUN_TEST(test_fields_array_defaults_collected);
    RUN_TEST(test_fields_settings_json_stringize);
    RUN_TEST(test_fields_array_rejects_malformed_tables);

    UNITY_END();
}
