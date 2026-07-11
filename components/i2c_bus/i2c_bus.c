/**
 * @file i2c_bus.c
 * @brief The shared I2C master bus (see include/i2c_bus.h).
 */
#include "i2c_bus.h"

#include "esp_log.h"

#include "log_manager.h"

static const char *TAG = "i2c_bus";

static i2c_master_bus_handle_t s_bus;

esp_err_t i2c_bus_init(void)
{
    static const log_descriptor_t LOG_DESC = { "i2c_bus", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC);

    if (s_bus != NULL)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t cfg =
    {
        .i2c_port = -1, /* auto-select */
        .sda_io_num = CONFIG_WICAN_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_WICAN_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* board has externals too */
    };

    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "bus create failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "bus up (SDA %d, SCL %d)", CONFIG_WICAN_I2C_SDA_GPIO,
             CONFIG_WICAN_I2C_SCL_GPIO);
    return ESP_OK;
}

esp_err_t i2c_bus_start(void)
{
    return (s_bus != NULL) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t i2c_bus_stop(void)
{
    return ESP_OK; /* the bus lives for the life of the firmware */
}

i2c_master_bus_handle_t i2c_bus_handle(void)
{
    return s_bus;
}

esp_err_t i2c_bus_add_device(uint8_t addr, uint32_t speed_hz,
                             i2c_master_dev_handle_t *out)
{
    if (s_bus == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t cfg =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = speed_hz,
    };

    return i2c_master_bus_add_device(s_bus, &cfg, out);
}

esp_err_t i2c_bus_probe(uint8_t addr)
{
    if (s_bus == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_probe(s_bus, addr, 50);
}
