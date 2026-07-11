// About — product/firmware info (the legacy About tab).

import { h, conn } from "../app.js";

export async function render(card) {
  const st = conn.status || {};
  card.append(
    h("dl", { class: "kv" },
      h("dt", {}, "Product"), h("dd", {}, "ESPNetLink — MeatPi Electronics"),
      h("dt", {}, "Firmware"), h("dd", {}, st.version || "—"),
      h("dt", {}, "Device ID"), h("dd", {}, h("code", {}, st.device_id || "—")),
      h("dt", {}, "Documentation"), h("dd", {},
        h("a", { href: "https://github.com/meatpiHQ/espnetlink-fw", target: "_blank", rel: "noopener" },
          "github.com/meatpiHQ/espnetlink-fw")),
      h("dt", {}, "Source"), h("dd", {},
        h("a", { href: "https://github.com/meatpiHQ/espnetlink-fw", target: "_blank", rel: "noopener" },
          "github.com/meatpiHQ/espnetlink-fw"))));
  return null;
}
