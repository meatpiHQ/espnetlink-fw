// Status page — the legacy label/value list, fed by /api/status +
// /api/lte + /api/gps (ESPNetLink's uplink and fix). Read-only, polls
// while visible.

import { api, h, conn, fmt } from "../app.js";

const POLL_MS = 4000;

function rows(pairs) {
  return pairs.filter(([, v]) => v !== undefined && v !== null)
    .flatMap(([k, v]) => [
      h("dt", {}, k),
      h("dd", {}, v.nodeType ? v : String(v)),
    ]);
}

export async function render(card) {
  const kv = h("dl", { class: "kv" });
  const memTitle = h("div", { class: "section-title sub", style: "margin-top:1.5rem" }, "Memory");
  const memTable = h("table", { class: "rtable" },
    h("thead", {}, h("tr", {},
      h("th", {}, "Region"), h("th", {}, "Free"),
      h("th", {}, "Largest block"), h("th", {}, "Min free"), h("th", {}, "Total"))),
    h("tbody", {}));
  card.append(kv, memTitle, memTable);

  const pillFor = (on, yes = "Connected", no = "Not Connected") =>
    h("span", { class: `status-indicator ${on ? "status-connected" : "status-neutral"}` },
      on ? yes : no);

  async function refresh() {
    let st = conn.status;
    try { st = await api("/api/status"); } catch { if (!st) return; }
    let lte = null, gps = null, vpn = null, imu = null;
    try { lte = await api("/api/lte"); } catch { /* degraded */ }
    try { gps = await api("/api/gps"); } catch { /* degraded */ }
    try { vpn = await api("/api/vpn"); } catch { /* not wired */ }
    try { imu = await api("/api/imu"); } catch { /* not wired */ }

    const bits = st.bits || {};
    kv.replaceChildren(...rows([
      ["WiFi Station", pillFor(bits.sta_connected)],
      ["Access Point", pillFor(bits.ap_enabled, "Enabled", "Disabled")],
      ["LTE", lte ? pillFor(lte.connected, "Connected",
                            lte.attached ? "Attached" : "Searching") : undefined],
      ["Operator", lte && lte.operator ? lte.operator : undefined],
      ["Network Type", lte && lte.network_type ? lte.network_type : undefined],
      ["Signal", lte && lte.rssi_dbm != null ? `${lte.rssi_dbm} dBm` : undefined],
      ["LTE IP", lte && lte.ip ? h("code", {}, lte.ip) : undefined],
      ["GPS Fix", gps ? pillFor(gps.fix, "Fix", "No fix") : undefined],
      ["Position", gps && gps.fix
        ? h("code", {}, `${gps.latitude.toFixed(6)}, ${gps.longitude.toFixed(6)}`)
        : undefined],
      ["Satellites", gps ? `${gps.satellites} tracked / ${gps.sats_in_view} in view` : undefined],
      ["Speed", gps && gps.fix && gps.speed_kmph != null
        ? `${gps.speed_kmph.toFixed(1)} km/h` : undefined],
      ["VPN", vpn && vpn.state !== "disabled"
        ? pillFor(vpn.state === "connected", "Connected", vpn.state) : undefined],
      ["VPN IP", vpn && vpn.state === "connected" && vpn.ts_ip
        ? h("code", {}, vpn.ts_ip) : undefined],
      ["Internet Time Synced", bits.time_synced ? "Yes" : "No"],
      ["Motion", imu ? (imu.activity === "active" ? "Active"
        : imu.activity === "stationary" ? "Stationary" : "Unknown")
        : (bits.motion ? "Active" : "Stationary")],
      ["Uptime", st.uptime],
      ["Firmware", st.version],
      ["Running Partition", st.partition],
      ["Boot Count", st.boot_count],
      ["Unexpected Resets", st.unexpected_resets],
    ]));

    const body = memTable.querySelector("tbody");
    const memRow = (label, m) => m && h("tr", {},
      h("td", { "data-k": "Region" }, label),
      h("td", { "data-k": "Free" }, fmt.bytes(m.free)),
      h("td", { "data-k": "Largest block" }, fmt.bytes(m.largest_block)),
      h("td", { "data-k": "Min free" }, fmt.bytes(m.min_free)),
      h("td", { "data-k": "Total" }, fmt.bytes(m.total)));
    if (st.memory) {
      body.replaceChildren(memRow("Internal RAM", st.memory.internal),
                           memRow("PSRAM", st.memory.psram));
      memTitle.style.display = memTable.style.display = "";
    } else {
      memTitle.style.display = memTable.style.display = "none";
    }
  }

  await refresh();
  const timer = setInterval(() => { if (conn.state === "online") refresh(); }, POLL_MS);
  return () => clearInterval(timer);
}
