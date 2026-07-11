// Settings — legacy layout: ONE page of sections (blue underlined
// titles) with label-left / control-right rows, ending in the sticky
// "Submit Changes" bar. Still fully SCHEMA-DRIVEN: every form comes
// from GET /api/settings/<name>/schema, friendly names/grouping from
// ../sections.js, unknown components auto-appear under Advanced.

import { api, h, toast, expectReboot, confirmDlg } from "../app.js";
import { SETTINGS_SECTIONS, SETTINGS_HIDDEN } from "../sections.js";

const SECRET_HINT = /password|passkey|token|key$/i;

const FIELD_LABELS = {
  sta_ssid: "SSID", sta_password: "Password",
  sta_auto_reconnect: "Auto Reconnect", sta_max_retry: "Connect Retries (-1 = forever)",
  ap_ssid: "AP SSID (blank = default)", ap_password: "AP Password",
  ap_channel: "AP Channel", ap_auto_disable: "Auto-Disable AP",
  mode: "Mode", hostname: "Hostname (blank = default)",
  broker_url: "Broker URL", broker_username: "Username", broker_password: "Password",
  client_id: "Client ID (blank = device ID)", topic_prefix: "Topic Prefix (blank = default)",
  keepalive_s: "Keep-Alive (s)", cert_set: "Certificate Set (TLS)",
  passkey: "Passkey", tx_power_dbm: "TX Power (dBm)",
  pairing_at_boot: "Allow Pairing After Power-On", conn_profile: "Connection Profile",
  enabled: "Enable", cli: "Console Command",
  std_enabled: "Standard PIDs", custom_enabled: "Custom PIDs", specific_enabled: "Vehicle Specific PIDs",
  std_init: "Standard Initialisation", custom_init: "Custom Initialisation",
  specific_init: "Specific Initialisation", std_protocol: "OBD Protocol",
  pause_below_mv: "Pause Polling Below (mV, 0 = never)", pause_mode: "Pause Behavior",
  min_event_interval_ms: "Event Rate Limit (ms)", vehicle: "Vehicle Model",
  wake_voltage_mv: "Wake Voltage (mV)", sleep_voltage_mv: "Sleep Voltage (mV)",
  sleep_time_min: "Sleep Delay (min)", baud: "Baud Rate",
  sta_ble_handover: "Pause WiFi While Bluetooth Connected",
  ap_ble_exclusive: "AP and Bluetooth Exclusive",
  url: "Broker URL",
  // vpn_manager
  type: "VPN Type", private_key: "Device Private Key (use Generate below)",
  peer_public_key: "Server Public Key", preshared_key: "Preshared Key (optional)",
  address: "Tunnel Address", allowed_ip: "Allowed IP", allowed_ip_mask: "Allowed IP Mask",
  endpoint: "Server Address", port: "Server Port",
  default_route: "Route All Traffic Through VPN", dns: "DNS Override (blank = off)",
  // data_logger
  format: "File Format", max_file_mb: "File Size Limit (MB)",
  max_files: "Files To Keep", batch_rows: "Write Batch (rows)",
  flush_ms: "Flush Interval (ms)",
  // sleep_manager
  sleep_mv: "Sleep Below (mV)", sleep_delay_min: "Sleep Delay (min)",
  periodic_wakeup: "Periodic Wake-Up", wakeup_interval_min: "Wake-Up Interval (min)",
};

/* per-component extra controls appended AFTER the schema rows */
const COMPONENT_EXTRAS = {
  vpn_manager(wrap) {
    const out = h("p", { class: "hint mono", style: "display:none" });
    wrap.append(
      h("div", { class: "form-row" },
        h("span", { class: "lbl" }, "Device Key Pair:"),
        h("div", {},
          h("button", { class: "btn small", type: "button", onclick: async (e) => {
            if (!(await confirmDlg(
              "Generate a new WireGuard key pair on the device? The old key stops working after the next restart.")))
              return;
            e.target.disabled = true;
            try {
              const r = await api("/api/vpn/keygen", { method: "POST" });
              out.textContent = "Public key (register it at your server): " + r.public_key;
              out.style.display = "";
              toast("Key stored on the device — applies after restart", "ok");
            } catch (err) { toast("Keygen failed: " + String(err.message), "err"); }
            e.target.disabled = false;
          } }, "Generate New Keys"),
          h("p", { class: "hint" },
            "The private key is created and stored on the device only — you never see or paste it."))),
      out);
  },
};

const label = (k) => FIELD_LABELS[k] ||
  k.replaceAll("_", " ").replace(/\b\w/g, (c) => c.toUpperCase());

const autoName = (n) => n.replace(/_manager$/, "").replaceAll("_", " ")
  .replace(/\b\w/g, (c) => c.toUpperCase());

/* ---- schema → legacy form rows ------------------------------------------------ */
function rowFor(key, spec, state, onDirty) {
  const set = (v) => { state[key] = v; onDirty(); };
  const cur = state[key];
  let control;

  if (spec.type === "boolean") {
    control = h("select", { onchange: (e) => set(e.target.value === "1") },
      h("option", { value: "1", ...(cur ? { selected: "" } : {}) }, "Enable"),
      h("option", { value: "0", ...(!cur ? { selected: "" } : {}) }, "Disable"));
  } else if (spec.enum) {
    control = h("select", { onchange: (e) => set(e.target.value) },
      ...spec.enum.map((v) =>
        h("option", { value: v, ...(v === cur ? { selected: "" } : {}) }, v)));
  } else if (spec.type === "integer" || spec.type === "number") {
    control = h("input", { type: "number", value: cur ?? "",
      ...(spec.minimum !== undefined ? { min: spec.minimum } : {}),
      ...(spec.maximum !== undefined ? { max: spec.maximum } : {}),
      onchange: (e) => set(e.target.value === "" ? 0 : Number(e.target.value)) });
  } else if (spec.type === "array") {
    return arrayRows(key, spec, state, onDirty);
  } else {
    const secret = SECRET_HINT.test(key);
    const input = h("input", {
      type: secret ? "password" : "text", value: cur ?? "",
      ...(spec.maxLength ? { maxlength: spec.maxLength } : {}),
      ...(secret ? { autocomplete: "new-password" } : {}),
      onchange: (e) => set(e.target.value),
    });
    control = secret
      ? h("div", { class: "row" }, h("div", { class: "grow" }, input),
          h("button", { class: "btn ghost small", type: "button",
            onclick: () => { input.type = input.type === "password" ? "text" : "password"; } },
            "show"))
      : input;
  }

  return h("div", { class: "form-row" },
    h("span", { class: "lbl" }, label(key) + ":"), control);
}

function arrayRows(key, spec, state, onDirty) {
  const items = Array.isArray(state[key]) ? state[key] : (state[key] = []);
  const itemProps = (spec.items && spec.items.properties) || null;
  const listEl = h("div", {});

  const rebuild = () => listEl.replaceChildren(...items.map((item, i) =>
    h("div", { class: "param-block" },
      h("div", { class: "row", style: "justify-content:space-between" },
        h("b", {}, `${label(key).replace(/s$/, "")} ${i + 1}`),
        h("button", { class: "delete-btn small btn", type: "button",
          onclick: () => { items.splice(i, 1); onDirty(); rebuild(); } }, "Delete")),
      itemProps
        ? Object.entries(itemProps).map(([ik, ispec]) => rowFor(ik, ispec, item, onDirty))
        : h("textarea", { rows: 4, onchange: (e) => {
            try { items[i] = JSON.parse(e.target.value); onDirty(); }
            catch { toast("Invalid JSON", "err"); }
          } }, JSON.stringify(item, null, 1)))));
  rebuild();

  return h("div", {},
    h("div", { class: "form-row" },
      h("span", { class: "lbl" }, label(key) +
        (spec.maxItems ? ` (max ${spec.maxItems}):` : ":")),
      h("div", {}, h("button", { class: "btn success small", type: "button",
        onclick: () => {
          if (spec.maxItems && items.length >= spec.maxItems)
            return toast(`Max ${spec.maxItems}`, "err");
          const blank = {};
          if (itemProps)
            for (const [ik, ispec] of Object.entries(itemProps))
              blank[ik] = ispec.type === "boolean" ? false
                        : (ispec.type === "integer" || ispec.type === "number") ? (ispec.minimum ?? 0)
                        : (ispec.enum ? ispec.enum[0] : "");
          items.push(blank); onDirty(); rebuild();
        } }, `Add ${label(key).replace(/s$/, "")}`))),
    listEl);
}

/* ---- page --------------------------------------------------------------------- */
export async function render(card) {
  let comps = [];
  try {
    const res = await api("/api/settings");
    comps = (res.components || res).filter((c) => !SETTINGS_HIDDEN.has(c.name));
  } catch (e) {
    card.append(h("p", {}, "Failed to list settings: ", String(e.message)));
    return null;
  }
  const byName = new Map(comps.map((c) => [c.name, c]));
  const placed = new Set(SETTINGS_SECTIONS.flatMap((s) => Object.keys(s.comps)));

  const edits = new Map();
  const submitBtn = h("button", { disabled: "" }, "Submit Changes");
  const dirtyNote = h("span", { class: "muted" }, "No unsaved changes");
  const markDirty = () => {
    submitBtn.disabled = edits.size === 0;
    dirtyNote.textContent = edits.size
      ? "Changes are saved to the device and applied by a restart"
      : "No unsaved changes";
  };

  async function componentBlock(comp, title, desc) {
    const wrap = h("div", {},
      h("div", { class: "section-title", style: "margin-top:1.5rem" }, title,
        comp.degraded ? h("span", { class: "status-indicator status-disconnected", style: "margin-left:.6rem" }, "check settings") : "",
        comp.pending_reboot ? h("span", { class: "status-indicator status-warn", style: "margin-left:.6rem" }, "restart pending") : ""),
      desc ? h("p", { class: "description-text" }, desc) : null);
    try {
      const [values, schema] = await Promise.all([
        api(`/api/settings/${comp.name}`),
        api(`/api/settings/${comp.name}/schema`),
      ]);
      delete values.degraded; delete values.pending_reboot;
      const props = (schema && schema.properties) || {};
      const onDirty = () => { edits.set(comp.name, values); markDirty(); };
      wrap.append(...Object.keys(props).map((k) => rowFor(k, props[k], values, onDirty)));
      if (COMPONENT_EXTRAS[comp.name]) COMPONENT_EXTRAS[comp.name](wrap);
    } catch (e) {
      wrap.append(h("p", { class: "description-text" }, "Unavailable: " + String(e.message)));
    }
    return wrap;
  }

  // legacy: one long page, section per component in friendly order
  for (const sec of SETTINGS_SECTIONS) {
    for (const [name, [title, desc]] of Object.entries(sec.comps)) {
      if (byName.has(name)) card.append(await componentBlock(byName.get(name), title, desc));
    }
  }
  for (const c of comps)
    if (!placed.has(c.name)) card.append(await componentBlock(c, autoName(c.name), ""));

  card.append(h("div", { class: "apply-bar" }, dirtyNote,
    h("span", { class: "right" }), submitBtn));

  submitBtn.addEventListener("click", async () => {
    submitBtn.disabled = true;
    try {
      for (const [name, values] of edits)
        await api(`/api/settings/${name}`, { method: "PUT", body: values });
      const r = await api("/api/settings/submit", { method: "POST" });
      if (r && r.reboot) expectReboot("Applying settings");
      else toast("Nothing changed", "ok");
      edits.clear(); markDirty();
    } catch (e) {
      toast("Submit failed: " + String(e.message), "err");
      submitBtn.disabled = false;
    }
  });

  return null;
}
