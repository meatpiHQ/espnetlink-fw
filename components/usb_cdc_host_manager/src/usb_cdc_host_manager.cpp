/**
 * @file usb_cdc_host_manager.cpp
 * @brief USB CDC Host Manager implementation for multi-interface CDC-ACM devices
 *
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* ============================================================================
 * Includes
 * ========================================================================= */

#include "usb_cdc_host_manager.h"

#include <atomic>
#include <memory>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"

using namespace esp_usb;

/* ============================================================================
 * Constants and Module State
 * ========================================================================= */

namespace {

static const char* TAG = "usb_cdc_mgr";

// USB host stack configuration
constexpr uint32_t USB_LIB_TASK_STACK_SIZE = 4096;
constexpr UBaseType_t USB_LIB_TASK_PRIORITY = 10;

// CDC-ACM device contexts - one per interface
static std::unique_ptr<CdcAcmDevice> s_vcp_if0;
static std::unique_ptr<CdcAcmDevice> s_vcp_if2;

// Initialization state flags (atomic for thread safety)
static std::atomic<bool> s_host_installed{false};
static std::atomic<bool> s_cdc_installed{false};
static std::atomic<bool> s_usb_task_running{false};

/* ============================================================================
 * Private Helper Functions
 * ========================================================================= */

/**
 * @brief USB Host library event handling task
 *
 * Continuously processes USB host library events in a dedicated task.
 * Handles device enumeration, connection/disconnection, and cleanup.
 *
 * @param arg Unused task argument
 */
static void usb_lib_task(void* arg)
{
    ESP_LOGI(TAG, "USB host lib task started");

    while (true) {
        uint32_t event_flags = 0;
        const esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_lib_handle_events failed: %d", err);
        }

        // Free all devices when no clients remain
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "No USB clients, freeing all devices");
            (void)usb_host_device_free_all();
        }

        // Log when all devices have been freed
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed");
        }
    }
}

/**
 * @brief Ensure VCP drivers are registered (idempotent)
 *
 * Registers all supported VCP driver implementations.
 * Safe to call multiple times - only registers once.
 */
static void ensure_vcp_drivers_registered()
{
    static bool registered = false;

    if (registered) {
        return;
    }

    ESP_LOGI(TAG, "Registering VCP drivers");
    VCP::register_driver<CH34x>();
    // Additional drivers can be registered here:
    // VCP::register_driver<CP210x>();
    // VCP::register_driver<FTDI>();

    registered = true;
}

/**
 * @brief Get device pointer for specified interface
 *
 * @param interface_idx Interface index (USB_CDC_IFACE_0 or USB_CDC_IFACE_2)
 * @return Pointer to CdcAcmDevice or nullptr if not opened
 */
static inline CdcAcmDevice* get_device(uint8_t interface_idx)
{
    if (interface_idx == USB_CDC_IFACE_0 && s_vcp_if0) {
        return s_vcp_if0.get();
    }
    if (interface_idx == USB_CDC_IFACE_2 && s_vcp_if2) {
        return s_vcp_if2.get();
    }
    return nullptr;
}

} // anonymous namespace

/* ============================================================================
 * Public API Implementation
 * ========================================================================= */

extern "C" esp_err_t usb_cdc_mgr_init(void)
{
    // Install USB Host library (idempotent)
    if (!s_host_installed.load()) {
        ESP_LOGI(TAG, "Installing USB Host library");

        usb_host_config_t host_config = {};
        host_config.skip_phy_setup = false;
        host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;

        ESP_RETURN_ON_ERROR(
            usb_host_install(&host_config),
            TAG,
            "Failed to install USB Host library"
        );

        s_host_installed.store(true);
    }

    // Start USB event handling task (idempotent)
    if (!s_usb_task_running.load()) {
        ESP_LOGI(TAG, "Creating USB event task");

        const BaseType_t result = xTaskCreate(
            usb_lib_task,
            "usb_lib",
            USB_LIB_TASK_STACK_SIZE,
            nullptr,
            USB_LIB_TASK_PRIORITY,
            nullptr
        );

        if (result != pdTRUE) {
            ESP_LOGE(TAG, "Failed to create USB library task");
            return ESP_FAIL;
        }

        s_usb_task_running.store(true);
    }

    // Install CDC-ACM driver (idempotent)
    if (!s_cdc_installed.load()) {
        ESP_LOGI(TAG, "Installing CDC-ACM driver");

        const esp_err_t err = cdc_acm_host_install(nullptr);

        // ESP_ERR_INVALID_STATE means already installed, which is OK
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install CDC-ACM driver: %d", err);
            return err;
        }

        s_cdc_installed.store(true);
    }

    // Register VCP drivers
    ensure_vcp_drivers_registered();

    ESP_LOGI(TAG, "USB CDC Manager initialized successfully");
    return ESP_OK;
}

extern "C" esp_err_t usb_cdc_mgr_open(uint16_t vid,
                                      uint16_t pid,
                                      uint8_t interface_idx,
                                      uint32_t connection_timeout_ms,
                                      size_t out_buffer_size,
                                      size_t in_buffer_size,
                                      usb_cdc_event_cb_t event_cb,
                                      usb_cdc_rx_cb_t rx_cb,
                                      void* user_arg)
{
    // Validate interface index
    ESP_RETURN_ON_FALSE(
        interface_idx == USB_CDC_IFACE_0 || interface_idx == USB_CDC_IFACE_2,
        ESP_ERR_INVALID_ARG,
        TAG,
        "Invalid interface index: %u (must be %u or %u)",
        interface_idx,
        USB_CDC_IFACE_0,
        USB_CDC_IFACE_2
    );

    // Ensure USB host stack and drivers are initialized
    ESP_RETURN_ON_ERROR(
        usb_cdc_mgr_init(),
        TAG,
        "USB CDC Manager initialization failed"
    );

    // Check if interface is already open
    if (get_device(interface_idx) != nullptr) {
        ESP_LOGW(TAG, "Interface %u already open, closing first", interface_idx);
        if (interface_idx == USB_CDC_IFACE_0) {
            s_vcp_if0.reset();
        } else {
            s_vcp_if2.reset();
        }
    }

    // Configure device parameters
    cdc_acm_host_device_config_t dev_config = {};
    dev_config.connection_timeout_ms = connection_timeout_ms;
    dev_config.out_buffer_size = out_buffer_size;
    dev_config.in_buffer_size = in_buffer_size;
    dev_config.event_cb = event_cb;
    dev_config.data_cb = rx_cb;
    dev_config.user_arg = user_arg;

    ESP_LOGI(TAG, "Opening USB CDC device: VID=0x%04X PID=0x%04X interface=%u",
             vid, pid, interface_idx);

    // Open the device using VCP abstraction
    std::unique_ptr<CdcAcmDevice> dev(VCP::open(vid, pid, &dev_config, interface_idx));

    if (!dev) {
        ESP_LOGE(TAG, "Failed to open USB CDC device on interface %u", interface_idx);
        return ESP_FAIL;
    }

    // Store device in appropriate slot
    if (interface_idx == USB_CDC_IFACE_0) {
        s_vcp_if0 = std::move(dev);
    } else {
        s_vcp_if2 = std::move(dev);
    }

    ESP_LOGI(TAG, "Successfully opened USB CDC device on interface %u", interface_idx);
    return ESP_OK;
}

extern "C" esp_err_t usb_cdc_mgr_set_line_coding(uint8_t interface_idx,
                                                 uint32_t baudrate,
                                                 uint8_t stop_bits,
                                                 uint8_t parity,
                                                 uint8_t data_bits)
{
    // Get device for specified interface
    CdcAcmDevice* dev = get_device(interface_idx);

    ESP_RETURN_ON_FALSE(
        dev != nullptr,
        ESP_ERR_INVALID_STATE,
        TAG,
        "Interface %u is not open",
        interface_idx
    );

    // Configure line coding parameters
    cdc_acm_line_coding_t line = {};
    line.dwDTERate = baudrate;
    line.bCharFormat = stop_bits;
    line.bParityType = parity;
    line.bDataBits = data_bits;

    ESP_LOGI(TAG, "Setting line coding on interface %u: %lu baud, %u data bits, %u stop bits, parity %u",
             interface_idx, baudrate, data_bits, stop_bits, parity);

    // Apply line coding using device's (possibly vendor-specific) implementation
    // Note: Control lines (DTR/RTS) are NOT toggled here
    const esp_err_t err = dev->line_coding_set(&line);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set line coding on interface %u: %d", interface_idx, err);
        return err;
    }

    ESP_LOGI(TAG, "Line coding configured successfully on interface %u", interface_idx);
    return ESP_OK;
}

extern "C" esp_err_t usb_cdc_mgr_tx(uint8_t interface_idx,
                                    const uint8_t* data,
                                    size_t len,
                                    uint32_t timeout_ms)
{
    // Validate parameters
    ESP_RETURN_ON_FALSE(
        data != nullptr,
        ESP_ERR_INVALID_ARG,
        TAG,
        "Data pointer is NULL"
    );

    ESP_RETURN_ON_FALSE(
        len > 0,
        ESP_ERR_INVALID_ARG,
        TAG,
        "Data length is zero"
    );

    // Get device for specified interface
    CdcAcmDevice* dev = get_device(interface_idx);

    ESP_RETURN_ON_FALSE(
        dev != nullptr,
        ESP_ERR_INVALID_STATE,
        TAG,
        "Interface %u is not open",
        interface_idx
    );

    // Transmit data (blocking operation)
    ESP_LOGD(TAG, "Transmitting %u bytes on interface %u (timeout: %lu ms)",
             len, interface_idx, timeout_ms);

    const esp_err_t err = dev->tx_blocking(const_cast<uint8_t*>(data), len, timeout_ms);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Transmission failed on interface %u: %d", interface_idx, err);
        return err;
    }

    ESP_LOGD(TAG, "Transmission successful on interface %u", interface_idx);
    return ESP_OK;
}

extern "C" void usb_cdc_mgr_close_all(void)
{
    ESP_LOGI(TAG, "Closing all USB CDC devices");

    if (s_vcp_if0) {
        ESP_LOGI(TAG, "Closing interface %u", USB_CDC_IFACE_0);
        s_vcp_if0.reset();
    }

    if (s_vcp_if2) {
        ESP_LOGI(TAG, "Closing interface %u", USB_CDC_IFACE_2);
        s_vcp_if2.reset();
    }

    ESP_LOGI(TAG, "All USB CDC devices closed");
}
