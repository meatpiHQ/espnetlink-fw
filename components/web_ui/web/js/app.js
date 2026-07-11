// ESPNetLink UI core (WiCAN v6 layout): API access, the connection watchdog (online /
// offline / rebooting — the UI behaves like an app, never needs a
// manual refresh), hash router, DOM + toast helpers.
//
// Pages live in js/pages/*.js and export { title, render(el) }.
// Keep this file SMALL — page logic belongs in the page modules.

import { icon } from "./icons.js";

/* ---- API base ----------------------------------------------------------
   Same-origin on the device. For browser-only development:
     ?api=http://10.42.0.62   → talk to a real device (needs its CORS off/LAN)
     ?mock=1                  → canned responses from js/mock.js            */
const params = new URLSearchParams(location.search);
export const API_BASE = params.get("api") || "";
export const MOCK = params.has("mock");

let mockFetch = null;
if (MOCK) {
  mockFetch = (await import("./mock.js")).mockFetch;
}

export async function api(path, opts = {}) {
  if (mockFetch) return mockFetch(path, opts);

  const res = await fetch(API_BASE + path, {
    headers: opts.body ? { "Content-Type": "application/json" } : undefined,
    ...opts,
    body: opts.body ? JSON.stringify(opts.body) : undefined,
  });
  const text = await res.text();
  let data = null;
  try { data = text ? JSON.parse(text) : null; } catch { data = text; }
  if (!res.ok) {
    const msg = (data && data.error) ? data.error : `HTTP ${res.status}`;
    throw new Error(msg);
  }
  return data;
}

/* ---- connection watchdog ------------------------------------------------
   Polls /api/status. States: connecting → online ⇄ offline, plus an
   EXPECTED-reboot state (settings apply, OTA) with its own banner text.
   On regaining the device, the current page re-renders automatically.  */
export const conn = {
  state: "connecting",
  status: null,              // last /api/status payload (pages may read it)
  _rebootUntil: 0,
  _listeners: new Set(),
  onChange(fn) { this._listeners.add(fn); },
};

export function expectReboot(reason = "Rebooting") {
  conn._rebootUntil = Date.now() + 90_000;   // give slow boots headroom
  setConn("reboot", `${reason} — waiting for the device to come back…`);
}

function setConn(state, bannerText) {
  const prev = conn.state;
  conn.state = state;

  const pill = document.getElementById("conn-pill");
  pill.dataset.state = state === "reboot" ? "reboot" : state;
  pill.textContent = { online: "online", offline: "offline",
                       connecting: "connecting", reboot: "rebooting" }[state];

  const banner = document.getElementById("offline-banner");
  if (state === "online" || state === "connecting") {
    banner.hidden = true;
  } else {
    banner.hidden = false;
    banner.dataset.kind = state === "reboot" ? "reboot" : "offline";
    document.getElementById("offline-text").textContent =
      bannerText || "Device offline — reconnecting…";
  }

  if (prev !== state) conn._listeners.forEach((fn) => fn(state, prev));
  if (prev !== "online" && state === "online" && prev !== "connecting") {
    toast("Device is back online", "ok");
    rerender();               // pages pick up post-reboot state
  }
}

async function tick() {
  try {
    const st = await api("/api/status");
    conn.status = st;
    const id = st.device_id || (st.info && st.info.device_id);
    if (id) document.getElementById("device-id").textContent = id;
    setConn("online");
  } catch {
    if (conn.state === "reboot" && Date.now() < conn._rebootUntil) {
      // keep the reboot banner (expected outage)
    } else {
      setConn("offline");
    }
  }
}
setInterval(tick, 3000);

/* ---- DOM helpers ------------------------------------------------------------ */
export function h(tag, attrs = {}, ...children) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") el.className = v;
    else if (k === "html") el.innerHTML = v;
    else if (k.startsWith("on")) el.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null) el.setAttribute(k, v);
  }
  for (const c of children.flat()) {
    if (c == null) continue;
    el.append(c.nodeType ? c : document.createTextNode(c));
  }
  return el;
}

export function toast(msg, kind = "") {
  const t = h("div", { class: `toast ${kind}` }, msg);
  document.getElementById("toasts").append(t);
  setTimeout(() => t.remove(), 3800);
}

export function confirmDlg(msg) {           // one place to swap in a nicer dialog later
  return Promise.resolve(window.confirm(msg));
}

export const fmt = {
  bytes(n) {
    if (n == null) return "—";
    if (n > 1048576) return (n / 1048576).toFixed(2) + " MB";
    if (n > 1024) return (n / 1024).toFixed(1) + " KB";
    return n + " B";
  },
  bool(b) { return b ? "yes" : "no"; },
};

/* ---- router --------------------------------------------------------------------- */
const routes = [
  { path: "status",   title: "Status",   icon: "activity", page: "status" },
  { path: "settings", title: "Settings", icon: "settings", page: "settings" },
  { path: "system",   title: "System",   icon: "cpu",      page: "system" },
  { path: "about",    title: "About",    icon: "info",     page: "about" },
];

const pageCache = new Map();
let current = null;          // { route, cleanup }

async function navigate() {
  const path = location.hash.replace(/^#\/?/, "").split("/")[0] || "status";
  const route = routes.find((r) => r.path === path) || routes[0];

  buildSidebar(route);
  document.body.classList.remove("nav-open");

  if (current && current.cleanup) current.cleanup();

  const view = document.getElementById("view");
  const card = h("div", { class: "tabcontent", style: "display:block" },
    h("div", { class: "section-title" }, route.title));
  view.replaceChildren(card);

  try {
    if (!pageCache.has(route.page)) {
      pageCache.set(route.page, await import(`./pages/${route.page}.js`));
    }
    const mod = pageCache.get(route.page);
    current = { route, cleanup: await mod.render(card) };
  } catch (e) {
    card.append(h("p", {}, "Failed to load page: ", h("code", {}, String(e.message || e))));
    current = { route, cleanup: null };
  }
}
export function rerender() { navigate(); }

/* ---- boot -------------------------------------------------------------------------- */
function buildSidebar(activeRoute) {
  const nav = document.getElementById("sidebar");
  const logo = h("img", { class: "logo", src: "logo.svg", alt: "MeatPi",
    onerror: (e) => e.target.replaceWith(
      h("span", { class: "logo-text" }, "MeatPi electronics")) });
  nav.replaceChildren(logo, ...routes.map((r) =>
    h("a", {
      href: `#/${r.path}`, "data-path": r.path,
      class: activeRoute && r.path === activeRoute.path ? "active" : "",
      html: icon(r.icon) + `<b>${r.title}</b>`,
    })));
}

document.getElementById("menu-btn").innerHTML = icon("menu");
document.getElementById("menu-btn").addEventListener("click", () =>
  document.body.classList.toggle("nav-open"));
document.getElementById("scrim").addEventListener("click", () =>
  document.body.classList.remove("nav-open"));
window.addEventListener("hashchange", navigate);

buildSidebar();
navigate();
tick();
