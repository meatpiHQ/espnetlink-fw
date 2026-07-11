# esp_wireguard — vendored component

Vendored 2026-07-07 from `components/legacy/esp_wireguard` (originally
the trombik/esp_wireguard port with reference-C crypto: blake2s,
chacha20poly1305, x25519, curve25519 smult). Upstream API untouched —
`esp_wireguard.h` (init/connect/set_default/peer_is_up/disconnect).
`examples/` dropped. Sole consumer: `vpn_manager`.
