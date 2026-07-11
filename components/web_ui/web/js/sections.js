// Settings sections — the ONE place that maps firmware components to
// user-friendly groups/names/descriptions. Used by the sidebar (nav
// children) and the Settings page. A component not listed here still
// appears automatically under Advanced.

export const SETTINGS_SECTIONS = [
  { id: "network",  title: "WiFi & Network", comps: {
    wifi_manager: ["WiFi", "Join your home/phone network and control the device's own access point."],
    mdns_manager: ["Network discovery", "How the device announces itself on the network (espnetlink_xxxx.local)."],
    vpn_manager: ["VPN (WireGuard)", "Reach the device (or your broker) through a WireGuard tunnel. Generate the device key below — the private key never leaves the device."],
  }},
  { id: "bluetooth", title: "Bluetooth", comps: {
    ble_manager: ["Bluetooth", "BLE pairing and connection behavior for phone apps."],
    interface_manager: ["Radio coexistence", "What happens to WiFi while a Bluetooth client is connected."],
  }},
  { id: "mqtt",     title: "MQTT", comps: {
    mqtt_manager: ["MQTT broker", "Publish vehicle data and receive commands through your broker."],
  }},
  { id: "obd",      title: "OBD", comps: {
    obd_chip: ["OBD adapter", "The OBD chip: baud rate, sleep/wake voltages."],
    autopid: ["Vehicle polling", "Master switch and behavior of automatic PID polling (tables are edited on the AutoPID page)."],
  }},
  { id: "datapath", title: "Data streams", comps: {
    socket_manager: ["TCP/UDP servers", "Raw network ports (e.g. the OBD TCP server used by scan apps)."],
    websocket_manager: ["WebSocket channels", "Browser/app streaming channels (/ws/…)."],
    bridge_manager: ["Bridges", "Which data source feeds which channel — one source per bridge."],
    data_logger: ["Data logger", "Record vehicle values to the SD card (files appear under /sd/logs; use a rule on the Events tab to choose what gets logged)."],
  }},
  { id: "device",   title: "Device", comps: {
    led_manager: ["LED", "Idle color and brightness."],
    battery_monitor: ["Battery monitor", "Voltage measurement options."],
    imu_manager: ["Motion sensor", "Bump and movement detection sensitivity."],
    rtc_manager: ["Time", "Clock sync (NTP) options."],
    sleep_manager: ["Sleep", "Power-save sleep when the car battery drops below a threshold; the device wakes by itself when the voltage recovers."],
  }},
  { id: "advanced", title: "Advanced", comps: {
    log_manager: ["Logging", "Log levels and destinations."],
    filesystem: ["Storage", "Internal storage options."],
    external_storage: ["SD card", "SD card options."],
    dev_status_manager: ["Status reporting", "Console command toggle."],
    restart_tracker: ["Restart history", "Console command toggle."],
    cmdline_manager: ["Console", "Command-line availability."],
  }},
];

// Components never shown on the Settings page (owned by another tab).
export const SETTINGS_HIDDEN = new Set(["event_manager"]);
