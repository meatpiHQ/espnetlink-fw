idf_component_register(
    SRCS "imu_manager_events.c" "imu_manager.c"
         "imu_manager_policy.c"
         "imu_manager_settings.c"
         "imu_manager_migrate.c"
         "imu_manager_http.c"
         "imu_manager_cli.c"
         "imu_manager_wom.c"
         "icm42670/icm42670.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "." "icm42670"
    # Feature: owns the ICM-42670 IMU on the shared bus (i2c_bus) —
    # SMD/WoM detectors POLLED via INT_STATUS2 (no ISR, no GPIO dep) ->
    # activity state + subscriber events. /api/imu and the `imu` command
    # are optional (main wires them). icm42670/ = vendored espressif
    # driver (PROVENANCE.md).
    # event_manager dropped in the espnetlink port: imu_manager_events.c
    # is kept on disk but excluded from SRCS (its call sites in
    # imu_manager.c are no-ops — same pattern as dev_status_manager).
    # hw_config + esp_driver_gpio: the ESPNetLink WoM deep-sleep extension
    # (imu_manager_wom.c) reads IMU_INT_PIN via gpio before EXT0 arm.
    PRIV_REQUIRES i2c_bus esp_driver_i2c esp_timer http_server_manager
                  dev_status_manager settings_manager log_manager
                  cmdline_manager hw_config esp_driver_gpio
)
