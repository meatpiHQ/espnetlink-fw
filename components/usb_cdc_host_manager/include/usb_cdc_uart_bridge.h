#pragma once

#include "esp_err.h"
#include "usb_cdc_host_manager.h" // for usb_cdc_event_cb_t, usb_cdc_rx_cb_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	// UART configuration
	int uart_port;              // 0, 1, 2
	int uart_tx_pin;            // -1 to keep current/no change
	int uart_rx_pin;            // -1 to keep current/no change
	bool use_hw_flow;           // RTS/CTS enable
	int uart_rts_pin;           // only if use_hw_flow
	int uart_cts_pin;           // only if use_hw_flow
	int uart_baud;              // e.g., 115200
	int uart_data_bits;         // 5..8
	int uart_parity;            // 0=None,1=Odd,2=Even
	int uart_stop_bits;         // 1 or 2

	// USB CDC device selection
	uint16_t usb_vid;           // e.g., 0x1A86
	uint16_t usb_pid;           // e.g., 0x55D2
	uint8_t usb_interface;      // 0 or 2

	// USB CDC line coding
	int usb_baud;               // DTE rate
	int usb_data_bits;          // 5..8
	int usb_parity;             // 0=None..4=Space
	int usb_stop_bits;          // 0=1,1=1.5,2=2

	// Buffers and timeouts
	int usb_conn_timeout_ms;    // open timeout
	int usb_tx_timeout_ms;      // write timeout
	size_t usb_out_buffer;      // OUT buffer size
	size_t usb_in_buffer;       // IN buffer size

	// Optional callbacks
	usb_cdc_event_cb_t event_cb;    // user event callback (can be NULL)
	usb_cdc_rx_cb_t rx_tap_cb;      // called on incoming USB data before forwarding to UART; can veto by returning false
	void *user_ctx;                 // passed to callbacks
} usb_cdc_uart_bridge_config_t;

// Start a UART <-> USB CDC bridge with the provided configuration.
// Bidirectional passthrough:
//  - UART->USB: a task reads UART and writes to USB
//  - USB->UART: RX callback writes to UART
esp_err_t usb_cdc_uart_bridge_start(const usb_cdc_uart_bridge_config_t *cfg);

// Stop the bridge and free resources. Safe to call even if not started.
void usb_cdc_uart_bridge_stop(void);

#ifdef __cplusplus
}
#endif
