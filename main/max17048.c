#include "max17048.h"

#include "app_config.h"
#include "i2c_bus.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX17048_REG_VCELL 0x02
#define MAX17048_REG_MODE 0x06
#define MAX17048_REG_SOC 0x04
#define MAX17048_REG_CRATE 0x16
#define MAX17048_REG_STATUS 0x1A
#define MAX17048_MODE_QUICKSTART 0x4000
#define MAX17048_CRATE_LSB_PERCENT_PER_HOUR 0.208f

static const char *TAG = APP_LOG_TAG;

static esp_err_t max17048_read_reg(max17048_gauge_t *gauge, uint8_t reg, uint16_t *value);
static esp_err_t max17048_write_reg(max17048_gauge_t *gauge, uint8_t reg, uint16_t value);
static esp_err_t max17048_read_voltage_v(max17048_gauge_t *gauge, float *voltage_v);
static esp_err_t max17048_read_soc_percent(max17048_gauge_t *gauge, float *soc_percent);
static esp_err_t max17048_read_crate_percent_per_hour(max17048_gauge_t *gauge, float *crate_percent_per_hour);
static esp_err_t max17048_read_status_raw(max17048_gauge_t *gauge, uint16_t *status_raw);

static esp_err_t max17048_read_reg(max17048_gauge_t *gauge, uint8_t reg, uint16_t *value)
{
    uint8_t rx[2] = {0};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(gauge->dev, &reg, 1, rx, sizeof(rx), 50),
        TAG,
        "max17048 read reg 0x%02X failed",
        reg);
    *value = (uint16_t)((rx[0] << 8) | rx[1]);
    return ESP_OK;
}

static esp_err_t max17048_write_reg(max17048_gauge_t *gauge, uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(gauge->dev, tx, sizeof(tx), 50),
        TAG,
        "max17048 write reg 0x%02X failed",
        reg);
    return ESP_OK;
}

static esp_err_t max17048_read_voltage_v(max17048_gauge_t *gauge, float *voltage_v)
{
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(max17048_read_reg(gauge, MAX17048_REG_VCELL, &raw), TAG, "max17048 vcell read failed");
    *voltage_v = (float)raw * 0.000078125f;
    return ESP_OK;
}

static esp_err_t max17048_read_soc_percent(max17048_gauge_t *gauge, float *soc_percent)
{
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(max17048_read_reg(gauge, MAX17048_REG_SOC, &raw), TAG, "max17048 soc read failed");
    *soc_percent = (float)raw / 256.0f;
    return ESP_OK;
}

static esp_err_t max17048_read_crate_percent_per_hour(max17048_gauge_t *gauge, float *crate_percent_per_hour)
{
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(max17048_read_reg(gauge, MAX17048_REG_CRATE, &raw), TAG, "max17048 crate read failed");
    *crate_percent_per_hour = (float)(int16_t)raw * MAX17048_CRATE_LSB_PERCENT_PER_HOUR;
    return ESP_OK;
}

static esp_err_t max17048_read_status_raw(max17048_gauge_t *gauge, uint16_t *status_raw)
{
    ESP_RETURN_ON_ERROR(max17048_read_reg(gauge, MAX17048_REG_STATUS, status_raw), TAG, "max17048 status read failed");
    return ESP_OK;
}

esp_err_t max17048_init(max17048_gauge_t *gauge)
{
    if (gauge->dev != NULL) {
        i2c_master_bus_rm_device(gauge->dev);
        gauge->dev = NULL;
    }

    ESP_RETURN_ON_ERROR(i2c_bus_attach_device(gauge->address, &gauge->dev), TAG, "max17048 attach failed");

    for (int i = 0; i < MAX17048_INIT_RETRIES; i++) {
        float voltage_v = 0.0f;
        float soc_percent = 0.0f;
        float crate_percent_per_hour = 0.0f;
        uint16_t status_raw = 0;

        if (max17048_read_voltage_v(gauge, &voltage_v) == ESP_OK &&
            max17048_read_soc_percent(gauge, &soc_percent) == ESP_OK &&
            max17048_read_crate_percent_per_hour(gauge, &crate_percent_per_hour) == ESP_OK &&
            max17048_read_status_raw(gauge, &status_raw) == ESP_OK) {
            gauge->voltage_v = voltage_v;
            gauge->soc_percent = soc_percent;
            gauge->crate_percent_per_hour = crate_percent_per_hour;
            gauge->status_raw = status_raw;
            gauge->has_sample = true;
            gauge->present = true;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(MAX17048_INIT_RETRY_DELAY_MS));
    }

    i2c_master_bus_rm_device(gauge->dev);
    gauge->dev = NULL;
    gauge->present = false;
    gauge->has_sample = false;
    return ESP_ERR_NOT_FOUND;
}

esp_err_t max17048_quickstart(max17048_gauge_t *gauge)
{
    if (gauge == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (gauge->dev == NULL) {
        ESP_RETURN_ON_ERROR(max17048_init(gauge), TAG, "max17048 quickstart init failed");
    }

    ESP_RETURN_ON_ERROR(
        max17048_write_reg(gauge, MAX17048_REG_MODE, MAX17048_MODE_QUICKSTART),
        TAG,
        "max17048 quickstart failed");

    gauge->present = true;
    gauge->has_sample = false;
    gauge->voltage_v = 0.0f;
    gauge->soc_percent = 0.0f;
    gauge->crate_percent_per_hour = 0.0f;
    gauge->status_raw = 0;
    gauge->last_sample_us = 0;
    gauge->last_retry_us = 0;
    return ESP_OK;
}

bool max17048_sample_update(max17048_gauge_t *gauge, int64_t now_us)
{
    if (!gauge->present) {
        return false;
    }
    if (gauge->last_sample_us != 0 && (now_us - gauge->last_sample_us) < MAX17048_SAMPLE_INTERVAL_US) {
        return false;
    }

    float voltage_v = 0.0f;
    float soc_percent = 0.0f;
    float crate_percent_per_hour = 0.0f;
    uint16_t status_raw = 0;

    if (max17048_read_voltage_v(gauge, &voltage_v) != ESP_OK) {
        return false;
    }
    if (max17048_read_soc_percent(gauge, &soc_percent) != ESP_OK) {
        return false;
    }
    if (max17048_read_crate_percent_per_hour(gauge, &crate_percent_per_hour) != ESP_OK) {
        return false;
    }
    if (max17048_read_status_raw(gauge, &status_raw) != ESP_OK) {
        return false;
    }

    gauge->voltage_v = voltage_v;
    gauge->soc_percent = soc_percent;
    gauge->crate_percent_per_hour = crate_percent_per_hour;
    gauge->status_raw = status_raw;
    gauge->has_sample = true;
    gauge->last_sample_us = now_us;
    return true;
}

void max17048_retry_if_needed(max17048_gauge_t *gauge, int64_t now_us)
{
    if (gauge->present) {
        return;
    }
    if (gauge->last_retry_us != 0 && (now_us - gauge->last_retry_us) < MAX17048_RETRY_INTERVAL_US) {
        return;
    }

    gauge->last_retry_us = now_us;
    if (max17048_init(gauge) == ESP_OK) {
        ESP_LOGI(TAG, "max17048 ready at 0x%02X", gauge->address);
    }
}
