#pragma once

#include <stdbool.h>

/**
 * @brief Register all built-in CLI commands with esp_console.
 *        Called once during console initialisation.
 */
void usb_cli_register_console_commands(void);

/**
 * @brief Return the current state of device-side input echo.
 *        Used by usb_cli_console.c to decide whether to echo typed characters.
 */
bool usb_cli_echo_is_enabled(void);
