#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    uint8_t address;
    const char *label;
    i2c_master_dev_handle_t dev;
    bool present;
    bool has_sample;
    float current_ma;
    float total_mah;
    int64_t last_sample_us;
    float last_emit_current_ma;
    float last_emit_total_mah;
    bool has_emitted_current;
    bool has_emitted_total;
} ina219_sensor_t;

esp_err_t ina219_init(ina219_sensor_t *sensor);
bool ina219_sample_update(ina219_sensor_t *sensor, int64_t now_us);
