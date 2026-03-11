#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t ssd1306_detect_and_init(void);
esp_err_t ssd1306_refresh_pages(const uint8_t *framebuffer, uint8_t first_page, uint8_t last_page);
uint8_t ssd1306_get_address(void);
