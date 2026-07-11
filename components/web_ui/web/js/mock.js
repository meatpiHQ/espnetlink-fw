// DEV-ONLY mock API (?mock=1): canned responses so the whole UI can be
// browsed with no device. NOT embedded in the firmware — keep it out of
// EMBED_FILES. Shapes mirror components/HTTP_API.md.

const state = {
  autopidGroupOn: true,
  rpm: 5324,
};

const schemas = {
  wifi_manager: { type: "object", properties: {
    mode: { type: "string", enum: ["ap", "sta", "apsta"] },
    sta_ssid: { type: "string", maxLength: 32 },
    sta_password: { type: "string", maxLength: 64 },
    sta_auto_reconnect: { type: "boolean" },
    sta_max_retry: { type: "integer", minimum: -1, maximum: 100 },
    ap_ssid: { type: "string", maxLength: 32 },
    ap_password: { type: "string", minLength: 8, maxLength: 64 },
    ap_channel: { type: "integer", minimum: 1, maximum: 13 },
  }},
  mqtt_manager: { type: "object", properties: {
    enabled: { type: "boolean" },
    broker_url: { type: "string", maxLength: 128 },
    broker_username: { type: "string", maxLength: 64 },
    broker_password: { type: "string", maxLength: 64 },
    keepalive_s: { type: "integer", minimum: 5, maximum: 3600 },
    cert_set: { type: "string", maxLength: 32 },
  }},
  bridge_manager: { type: "object", properties: {
    bridges: { type: "array", maxItems: 4, items: { type: "object", properties: {
      name: { type: "string", maxLength: 15 },
      a: { type: "string", maxLength: 15 },
      b: { type: "string", maxLength: 15 },
      translator: { type: "string", maxLength: 15 },
      enabled: { type: "boolean" },
    }}},
  }},
  autopid: { type: "object", properties: {
    enabled: { type: "boolean" },
    std_enabled: { type: "boolean" },
    custom_enabled: { type: "boolean" },
    specific_enabled: { type: "boolean" },
    std_protocol: { type: "string", maxLength: 7 },
    pause_below_mv: { type: "integer", minimum: 0, maximum: 30000 },
    pause_mode: { type: "string", enum: ["all", "requests_only"] },
    min_event_interval_ms: { type: "integer", minimum: 10, maximum: 600000 },
    cli: { type: "boolean" },
  }},
  vpn_manager: { type: "object", properties: {
    enabled: { type: "boolean" },
    type: { type: "string", enum: ["wireguard"] },
    private_key: { type: "string", maxLength: 63 },
    peer_public_key: { type: "string", maxLength: 63 },
    preshared_key: { type: "string", maxLength: 63 },
    address: { type: "string", maxLength: 31 },
    allowed_ip: { type: "string", maxLength: 31 },
    allowed_ip_mask: { type: "string", maxLength: 31 },
    endpoint: { type: "string", maxLength: 63 },
    port: { type: "integer", minimum: 1, maximum: 65535 },
    keepalive_s: { type: "integer", minimum: 0, maximum: 3600 },
    default_route: { type: "boolean" },
    dns: { type: "string", maxLength: 15 },
    cli: { type: "boolean" },
  }},
  data_logger: { type: "object", properties: {
    enabled: { type: "boolean" },
    format: { type: "string", enum: ["sqlite", "csv", "binary"] },
    max_file_mb: { type: "integer", minimum: 1, maximum: 32 },
    max_files: { type: "integer", minimum: 1, maximum: 500 },
    batch_rows: { type: "integer", minimum: 16, maximum: 1024 },
    flush_ms: { type: "integer", minimum: 100, maximum: 60000 },
    cli: { type: "boolean" },
  }},
  sleep_manager: { type: "object", properties: {
    enabled: { type: "boolean" },
    sleep_mv: { type: "integer", minimum: 8000, maximum: 15000 },
    sleep_delay_min: { type: "integer", minimum: 1, maximum: 720 },
    periodic_wakeup: { type: "boolean" },
    wakeup_interval_min: { type: "integer", minimum: 1, maximum: 1440 },
    cli: { type: "boolean" },
  }},
};

const values = {
  wifi_manager: { mode: "apsta", sta_ssid: "MyHomeWiFi", sta_password: "",
    sta_auto_reconnect: true, sta_max_retry: -1, ap_ssid: "", ap_password: "12345678", ap_channel: 6 },
  mqtt_manager: { enabled: true, broker_url: "mqtts://broker.local:8883",
    broker_username: "car", broker_password: "", keepalive_s: 120, cert_set: "" },
  bridge_manager: { bridges: [
    { name: "br_obd", a: "obd", b: "ws_obd", translator: "raw", enabled: true },
    { name: "br_cli", a: "cli", b: "ws_cli", translator: "raw", enabled: true } ] },
  autopid: { enabled: true, std_enabled: true, custom_enabled: true, specific_enabled: true,
    std_protocol: "6", pause_below_mv: 0, pause_mode: "requests_only",
    min_event_interval_ms: 1000, cli: true },
  vpn_manager: { enabled: false, type: "wireguard", private_key: "",
    peer_public_key: "mockPeerPublicKey000000000000000000000000000=",
    preshared_key: "", address: "10.66.0.2", allowed_ip: "10.66.0.2",
    allowed_ip_mask: "255.255.255.0", endpoint: "vpn.example.com", port: 51820,
    keepalive_s: 25, default_route: false, dns: "", cli: true },
  data_logger: { enabled: true, format: "sqlite", max_file_mb: 4, max_files: 100,
    batch_rows: 256, flush_ms: 1000, cli: true },
  sleep_manager: { enabled: true, sleep_mv: 13100, sleep_delay_min: 3,
    periodic_wakeup: false, wakeup_interval_min: 30, cli: true },
  event_manager: { enabled: true,
    timers: [{ name: "beat", period_s: 15 }],
    rules: [
      { name: "high_rpm", on: "autopid.param", match: { param: "rpm" },
        when: [{ key: "value", op: ">", val: 3000 }],
        do: "mqtt.publish", with: { topic: "~/alerts", payload: "{\"rpm\":${value}}" },
        cooldown_ms: 60000 },
      { name: "beat_data", on: "timer.tick", match: { timer: "beat" },
        do: "mqtt.publish", with: { topic: "~/autopid", payload: "${autopid.data}" } },
    ] },
};

function autopidNow() {
  state.rpm = 5000 + Math.round(Math.random() * 800);
  return {
    groups: [{ name: "default", enabled: state.autopidGroupOn, period_ms: 1000 }],
    params: [
      { name: "rpm", unit: "rpm", value: state.rpm, ts_us: 1e6 },
      { name: "coolant_temp", unit: "C", value: 88, ts_us: 1e6 },
      { name: "VehicleSpeed", unit: "km/h", value: 64, ts_us: 1e6 },
      { name: "SOC", unit: "%", value: 71.5, ts_us: 1e6 },
    ],
    stats: { running: state.autopidGroupOn, paused_voltage: false, polls_ok: 4213,
      polls_failed: 3, pids: 4, filters: 1, period_floor_ms: 50, sub_floor_pids: 0,
      now_us: 2e6 },
  };
}

const GET = {
  "/api/status": () => ({
    bits: { awake: true, sta_connected: true, ap_enabled: false, ble_enabled: true,
      mqtt_connected: true, sdcard_mounted: true, time_synced: true,
      autopid_enabled: true, motion: false, sta_suspended: false },
    network_connected: true, uptime: "02:14:36", version: "v6.0.0-mock",
    partition: "ota_0", device_id: "14c19f44e349", boot_count: 42, unexpected_resets: 0,
    memory: {
      internal: { total: 318000, free: 131072, min_free: 98304, largest_block: 65536 },
      psram: { total: 8388608, free: 6815744, min_free: 6291456, largest_block: 4194304 } },
  }),
  "/api/battery": () => ({ voltage: 12.42 }),
  "/api/vpn": () => (values.vpn_manager.enabled
    ? { state: "connected", endpoint: "vpn.example.com:51820",
        connects: 1, failures: 0, uptime_s: 900 }
    : { state: "disabled", endpoint: "", connects: 0, failures: 0, uptime_s: 0 }),
  "/api/logger": () => ({ enabled: values.data_logger.enabled, running: true,
    paused: false, storage_ok: true, file: "dl_1783361504.db", file_rows: 2000,
    files: 5, queued: 3, written: 61864, dropped: 0, errors: 0, rotations: 1,
    dir: "/sd/logs" }),
  "/api/sleep": () => ({ enabled: values.sleep_manager.enabled,
    state: "low_voltage", voltage: 12.42, sleep_v: 13.1, wake_v: 13.2, naps: 0 }),
  "/api/settings": () => ({ components: Object.keys(values).map((name) => ({
    name, version: 1, degraded: false, pending_reboot: false })) }),
  "/api/autopid": autopidNow,
  "/api/autopid/data": () => ({ rpm: state.rpm, coolant_temp: 88, VehicleSpeed: 64, SOC: 71.5 }),
  "/api/autopid/config": () => ({
    groups: [{ name: "default", enabled: true, period_ms: 1000 }],
    pids: [
      { name: "engine", type: "std", cmd: "010C", group: "default",
        parameters: [{ name: "rpm", expression: "[B2:B3]*0.25", unit: "rpm" }] },
      { name: "MEB_SOC", type: "specific", cmd: "22028C",
        init: "ATSP7;ATCP17;ATSHFC007B;ATFCSH17FC007B;ATCRA17FE007B",
        parameters: [{ name: "SOC", expression: "(B3/2.5)", unit: "%" }] } ],
    filters: [{ frame_id: 291, monitor_ms: 1000, period_ms: 3000,
      parameters: [{ name: "flt_b0", expression: "B0" }] }],
  }),
  "/api/autopid/std_scan": () => ({ status: "done", found: 106, ts: 1783300000, stored: true }),
  "/api/autopid/std_scan/result": () => ({ version: 1, protocol: "6", found: 3, supported: [
    { pid: 12, cmd: "010C", name: "EngineRPM", parameters: [
      { name: "EngineRPM", expression: "[B2:B3]*0.25", unit: "rpm", class: "speed", min: 0, max: 16384 }] },
    { pid: 13, cmd: "010D", name: "VehicleSpeed", parameters: [
      { name: "VehicleSpeed", expression: "B2", unit: "km/h", class: "speed", min: 0, max: 255 }] },
    { pid: 5, cmd: "0105", name: "EngineCoolantTemp", parameters: [
      { name: "EngineCoolantTemp", expression: "B2-40", unit: "degC", class: "temperature" }] } ] }),
  "/api/events/sources": () => ([
    { event: "autopid.param", description: "a parameter changed", keys: [
      { key: "param", type: "string" }, { key: "value", type: "number" },
      { key: "unit", type: "string" }, { key: "group", type: "string" }] },
    { event: "timer.tick", description: "settings-defined periodic timers",
      keys: [{ key: "timer", type: "string" }] },
    { event: "mqtt.rx", description: "message on a rule-matched topic",
      keys: [{ key: "topic", type: "string" }, { key: "payload", type: "string" }] },
    { event: "battery.threshold", description: "battery crossed a threshold",
      keys: [{ key: "edge", type: "string" }, { key: "volts", type: "number" }] },
    { event: "status.bit", description: "a device-status bit changed",
      keys: [{ key: "bit", type: "string" }, { key: "set", type: "boolean" }] },
    { event: "imu.bump", description: "wake-on-motion pulse",
      keys: [{ key: "axes", type: "integer" }] },
    { event: "vpn.state", description: "WireGuard tunnel state changed",
      keys: [{ key: "connected", type: "string" }] },
    { event: "logger.rotated", description: "a log file reached its size cap",
      keys: [{ key: "file", type: "string" }] },
    { event: "sleep.entering", description: "about to power down (last chance to publish)",
      keys: [{ key: "volts", type: "number" }] },
    { event: "sleep.state", description: "sleep ladder transition",
      keys: [{ key: "state", type: "string" }, { key: "volts", type: "number" }] } ]),
  "/api/events/actions": () => ([
    { name: "mqtt.publish", params_schema: { type: "object", properties: {
      topic: { type: "string", minLength: 1, maxLength: 127 }, payload: { type: "string" },
      qos: { type: "integer", minimum: 0, maximum: 2 }, retain: { type: "boolean" } },
      required: ["topic", "payload"] } },
    { name: "log.note", params_schema: { type: "object", properties: {
      message: { type: "string" } } } },
    { name: "autopid.group", params_schema: { type: "object", properties: {
      group: { type: "string", minLength: 1 }, enabled: { type: "boolean" },
      period_ms: { type: "integer", minimum: 0 } }, required: ["group", "enabled"] } },
    { name: "led.indicate", params_schema: { type: "object", properties: {
      r: { type: "integer", minimum: 0, maximum: 255 }, g: { type: "integer", minimum: 0, maximum: 255 },
      b: { type: "integer", minimum: 0, maximum: 255 },
      mode: { type: "string", enum: ["solid", "off", "blink_slow", "blink_fast"] } },
      required: ["r", "g", "b"] } },
    { name: "http.post", params_schema: { type: "object", properties: {
      url: { type: "string", minLength: 1 }, body: { type: "string" },
      content_type: { type: "string" } }, required: ["url", "body"] } },
    { name: "obd.request", params_schema: { type: "object", properties: {
      cmd: { type: "string", minLength: 1, maxLength: 23 } }, required: ["cmd"] } },
    { name: "logger.enable", params_schema: { type: "object", properties: {} } },
    { name: "logger.disable", params_schema: { type: "object", properties: {} } },
    { name: "logger.write", params_schema: { type: "object", properties: {
      source: { type: "string", maxLength: 15 }, name: { type: "string", maxLength: 31 },
      value: { type: "string" } }, required: ["name", "value"] } } ]),
  "/api/events/values": () => (["autopid.data", "autopid.", "battery.voltage", "time.iso", "time.epoch"]),
  "/api/events/log": () => ({
    stats: { running: true, published: 231, dropped: 0, fired: 58, action_errors: 4, suppressed: 2 },
    events: [
      { source: "autopid", name: "param", ts_us: 100000,
        data: { param: "rpm", value: state.rpm, unit: "rpm", group: "default" }, fired: ["high_rpm"] },
      { source: "timer", name: "tick", ts_us: 200000, data: { timer: "beat" }, fired: ["beat_data"] },
      { source: "status", name: "bit", ts_us: 300000, data: { bit: "sta_connected", set: true }, fired: [] } ] }),
  "/api/logs": () => ({ lines: ["I (1000) main: mock log line", "W (2000) autopid: mock warning"] }),
};

export async function mockFetch(path, opts = {}) {
  await new Promise((r) => setTimeout(r, 120));   // feel like a network
  const method = (opts.method || "GET").toUpperCase();
  const clean = path.split("?")[0];

  if (method === "GET") {
    if (clean.startsWith("/api/settings/") && clean.endsWith("/schema"))
      return schemas[clean.split("/")[3]] || { type: "object", properties: {} };
    if (clean.startsWith("/api/settings/") && clean.split("/").length === 4)
      return structuredClone(values[clean.split("/")[3]] || {});
    if (GET[clean]) return GET[clean]();
  }
  if (method === "PUT" && clean.startsWith("/api/settings/")) {
    Object.assign(values[clean.split("/")[3]] ||= {}, opts.body);
    return { changed: true };
  }
  if (method === "PUT" && clean === "/api/autopid/config") return { ok: true, pids: 2, params: 3 };
  if (method === "POST" && clean === "/api/settings/submit") return { reboot: true };
  if (method === "POST" && clean === "/api/autopid/std_scan") return { started: true };
  if (method === "POST" && clean === "/api/vpn/keygen")
    return { public_key: "mockDevicePublicKey0000000000000000000000000=",
             pending_reboot: true };
  if (method === "POST" && clean === "/api/autopid/test")
    return { ok: true, elapsed_ms: 84.2, raw: "41 0C 53 30", payload: "41 0C 53 30", value: 5324 };

  throw new Error(`mock: no handler for ${method} ${clean}`);
}
