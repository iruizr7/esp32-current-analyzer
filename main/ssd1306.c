#include "ssd1306.h"

#include <string.h>

#include "app_config.h"
#include "i2c_bus.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = APP_LOG_TAG;
static i2c_master_dev_handle_t s_oled_dev = NULL;
static uint8_t s_oled_addr = SSD1306_ADDR_1;

static esp_err_t ssd1306_send_cmd(uint8_t cmd)
{
    const uint8_t tx[2] = {0x00, cmd};
    return i2c_master_transmit(s_oled_dev, tx, sizeof(tx), 50);
}

static esp_err_t ssd1306_send_data(const uint8_t *data, size_t len)
{
    uint8_t tx[1 + SSD1306_WIDTH];

    if (len > SSD1306_WIDTH) {
        return ESP_ERR_INVALID_SIZE;
    }

    tx[0] = 0x40;
    memcpy(&tx[1], data, len);
    return i2c_master_transmit(s_oled_dev, tx, len + 1, 50);
}

esp_err_t ssd1306_detect_and_init(void)
{
    static const uint8_t probe[] = {SSD1306_ADDR_1, SSD1306_ADDR_2};
    static const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF,
    };

    for (size_t i = 0; i < ARRAY_SIZE(probe); i++) {
        if (i2c_master_probe(i2c_bus_get(), probe[i], 30) != ESP_OK) {
            continue;
        }

        ESP_RETURN_ON_ERROR(i2c_bus_attach_device(probe[i], &s_oled_dev), TAG, "oled attach failed");
        s_oled_addr = probe[i];

        for (size_t k = 0; k < ARRAY_SIZE(init_cmds); k++) {
            ESP_RETURN_ON_ERROR(ssd1306_send_cmd(init_cmds[k]), TAG, "oled init cmd failed");
        }
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t ssd1306_refresh_pages(const uint8_t *framebuffer, uint8_t first_page, uint8_t last_page)
{
    if (framebuffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (last_page >= SSD1306_PAGES || first_page > last_page) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t page = first_page; page <= last_page; page++) {
        ESP_RETURN_ON_ERROR(ssd1306_send_cmd(0xB0 | page), TAG, "set page failed");
        ESP_RETURN_ON_ERROR(ssd1306_send_cmd(0x00), TAG, "set col low failed");
        ESP_RETURN_ON_ERROR(ssd1306_send_cmd(0x10), TAG, "set col high failed");
        ESP_RETURN_ON_ERROR(
            ssd1306_send_data(&framebuffer[page * SSD1306_WIDTH], SSD1306_WIDTH),
            TAG,
            "page write failed");
    }

    return ESP_OK;
}

uint8_t ssd1306_get_address(void)
{
    return s_oled_addr;
}
