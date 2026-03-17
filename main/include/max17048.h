#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define MAX17048_STATUS_SC 0x20
#define MAX17048_STATUS_SL 0x10
#define MAX17048_STATUS_VR 0x08
#define MAX17048_STATUS_VL 0x04
#define MAX17048_STATUS_VH 0x02
#define MAX17048_STATUS_RI 0x01

typedef struct {
    uint8_t address;
    i2c_master_dev_handle_t dev;
    bool present;
    bool has_sample;
    float voltage_v;
    float soc_percent;
    float crate_percent_per_hour;
    uint16_t status_raw;
    int64_t last_sample_us;
    int64_t last_retry_us;
} max17048_gauge_t;

esp_err_t max17048_init(max17048_gauge_t *gauge);
esp_err_t max17048_quickstart(max17048_gauge_t *gauge);
bool max17048_sample_update(max17048_gauge_t *gauge, int64_t now_us);
void max17048_retry_if_needed(max17048_gauge_t *gauge, int64_t now_us);
