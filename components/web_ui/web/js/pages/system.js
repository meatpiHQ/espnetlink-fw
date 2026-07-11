// System page: firmware update (OTA), settings backup/restore, factory
// reset, recent logs, about. All flows ride the reboot-aware watchdog —
// the page tells the user what is happening instead of asking them to
// refresh.

import { api, API_BASE, h, toast, expectReboot, confirmDlg } from "../app.js";
import { icon } from "../icons.js";

function otaCard() {
  const file = h("input", { type: "file", accept: ".bin" });
  const prog = h("progress", { max: 100, value: 0, style: "width:100%; display:none" });
  const note = h("p", { class: "hint" });

  return h("div", {},
    h("div", { class: "section-title sub" }, "Firmware Update"),
    h("p", { class: "description-text" }, "Upload an espnetlink-fw .bin — the device flashes the inactive slot and restarts. Do not power off during the update."),
    h("label", { class: "field" }, file), prog, note,
    h("div", { class: "row" },
      h("button", { html: icon("upload") + "<span>Flash</span>",
        onclick: async (e) => {
          const f = file.files && file.files[0];
          if (!f) return toast("Choose a .bin first", "err");
          if (!(await confirmDlg(`Flash ${f.name} (${(f.size / 1048576).toFixed(2)} MB)?`))) return;
          const btn = e.target.closest("button"); btn.disabled = true;
          prog.style.display = ""; note.textContent = "Uploading…";
          try {
            // XHR for upload progress (fetch has no upload events)
            await new Promise((resolve, reject) => {
              const xhr = new XMLHttpRequest();
              xhr.open("POST", API_BASE + "/api/ota/upload");
              xhr.upload.onprogress = (ev) =>
                (prog.value = ev.lengthComputable ? (ev.loaded / ev.total) * 100 : 0);
              xhr.onload = () => (xhr.status < 300 ? resolve() : reject(new Error(xhr.responseText || xhr.status)));
              xhr.onerror = () => reject(new Error("upload failed"));
              xhr.setRequestHeader("Content-Type", "application/octet-stream");
              xhr.send(f);
            });
            note.textContent = "Flashed — restarting into the new firmware.";
            expectReboot("Firmware update");
          } catch (err) {
            note.textContent = "";
            toast("OTA failed: " + String(err.message), "err");
          }
          btn.disabled = false;
        } })));
}

function backupCard() {
  const restoreFile = h("input", { type: "file", accept: ".json" });
  return h("div", { style: "margin-top:1.5rem" },
    h("div", { class: "section-title sub" }, "Settings Backup"),
    h("p", { class: "description-text" },
      "Download every component's settings (INCLUDING passwords — store it safely) or restore a backup to this or another device."),
    h("div", { class: "row" },
      h("button", { class: "btn ghost", html: icon("download") + "<span>Download Backup</span>",
        onclick: async () => {
          try {
            const data = await api("/api/settings/backup");
            const blob = new Blob([JSON.stringify(data, null, 1)], { type: "application/json" });
            const a = h("a", { href: URL.createObjectURL(blob),
              download: `espnetlink-settings-${new Date().toISOString().slice(0, 10)}.json` });
            a.click(); URL.revokeObjectURL(a.href);
          } catch (e) { toast(String(e.message), "err"); }
        } }),
      h("div", { class: "grow" }, restoreFile),
      h("button", { class: "btn ghost", html: icon("upload") + "<span>Restore</span>",
        onclick: async () => {
          const f = restoreFile.files && restoreFile.files[0];
          if (!f) return toast("Choose a backup file", "err");
          if (!(await confirmDlg("Restore ALL settings from this backup? The device restarts if anything changed."))) return;
          try {
            const body = JSON.parse(await f.text());
            const r = await api("/api/settings/backup", { method: "POST", body });
            if (r && r.reboot) expectReboot("Restoring settings");
            else toast(`Restored ${r.restored ?? "?"} components (no changes)`, "ok");
          } catch (e) { toast("Restore failed: " + String(e.message), "err"); }
        } })));
}

function dangerCard() {
  const confirmInput = h("input", { placeholder: 'type "factory-reset" to confirm' });
  return h("div", { style: "margin-top:1.5rem" },
    h("div", { class: "section-title sub" }, "Factory Reset"),
    h("p", { class: "description-text" },
      "Wipes ALL settings to defaults (stored files, certificates and the SD card are kept). The device restarts on its setup access point."),
    h("div", { class: "row" },
      h("div", { class: "grow" }, confirmInput),
      h("button", { class: "btn danger", html: icon("alert") + "<span>Factory Reset</span>",
        onclick: async () => {
          if (confirmInput.value.trim() !== "factory-reset")
            return toast('Type "factory-reset" first', "err");
          try {
            await api("/api/settings/factory_reset", { method: "POST", body: { confirm: "factory-reset" } });
            expectReboot("Factory reset");
          } catch (e) { toast(String(e.message), "err"); }
        } })));
}

function rebootCard() {
  return h("div", { style: "margin-top:1.5rem" },
    h("div", { class: "section-title sub" }, "Restart"),
    h("div", { class: "row" },
      h("button", { class: "btn ghost", html: icon("refresh") + "<span>Restart Device</span>",
        onclick: async () => {
          if (!(await confirmDlg("Restart the device now?"))) return;
          try {
            await api("/api/restart", { method: "POST" });
            expectReboot("Restarting");
          } catch (e) { toast(String(e.message), "err"); }
        } })));
}

async function logsCard() {
  const body = h("pre", { class: "term", style: "height:38vh" });
  const card = h("div", { style: "margin-top:1.5rem" },
    h("div", { class: "section-title-row" },
      h("div", { class: "section-title sub" }, "Recent Logs"),
      h("div", { class: "section-title-actions" },
        h("button", { class: "btn ghost small", html: icon("refresh"), onclick: refresh }))),
    body);
  async function refresh() {
    try {
      // the PSRAM crash ring (text/plain, chronological, spans reboots)
      const text = await api("/api/logs/ring");
      body.textContent = (typeof text === "string" && text) ? text : "(empty)";
      body.scrollTop = body.scrollHeight;
    } catch (e) { body.textContent = "unavailable: " + String(e.message); }
  }
  await refresh();
  return card;
}

export async function render(view) {
  view.append(otaCard(), backupCard(), rebootCard(), dangerCard(), await logsCard());
  return null;
}
