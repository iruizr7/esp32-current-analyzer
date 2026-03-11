#include "i2c_bus.h"

#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = APP_LOG_TAG;
static i2c_master_bus_handle_t s_i2c_bus = NULL;

esp_err_t i2c_bus_init(void)
{
    const i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &s_i2c_bus), TAG, "i2c_new_master_bus failed");
    return ESP_OK;
}

esp_err_t i2c_bus_attach_device(uint8_t addr, i2c_master_dev_handle_t *dev)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    return i2c_master_bus_add_device(s_i2c_bus, &cfg, dev);
}

void i2c_bus_scan(void)
{
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 20) == ESP_OK) {
            ESP_LOGI(TAG, "I2C device at 0x%02X", addr);
        }
    }
}

i2c_master_bus_handle_t i2c_bus_get(void)
{
    return s_i2c_bus;
}
