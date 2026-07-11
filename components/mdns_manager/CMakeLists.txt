idf_component_register(
    SRCS "mdns_manager.c"
         "mdns_manager_policy.c"
         "mdns_manager_settings.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Feature: the mDNS advertisement (device-contract v2:
    # _meatpi._tcp:80, espnetlink_<id> hostname, device_type/device_id/
    # mac/fw/api TXT). esp-mdns is a MANAGED component (see
    # idf_component.yml).
    # json (bundled cJSON on IDF v5.5) is used by mdns_manager_settings.c's
    # on_apply; the espressif/cjson managed component is IDF v6 only.
    PRIV_REQUIRES espressif__mdns dev_status_manager settings_manager
                  log_manager json
)
