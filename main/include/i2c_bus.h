#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t i2c_bus_init(void);
esp_err_t i2c_bus_attach_device(uint8_t addr, i2c_master_dev_handle_t *dev);
void i2c_bus_scan(void);
i2c_master_bus_handle_t i2c_bus_get(void);
