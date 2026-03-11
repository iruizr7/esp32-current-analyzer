#include "ina219.h"

#include "app_config.h"
#include "i2c_bus.h"
#include "esp_check.h"
#include "esp_log.h"

#define INA219_REG_CONFIG 0x00
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05
#define INA219_CONFIG_32V_2A 0x399F
#define INA219_CAL_32V_2A 0x1000
#define INA219_CURRENT_LSB_MA 0.1f

static const char *TAG = APP_LOG_TAG;

static esp_err_t ina219_write_reg(ina219_sensor_t *sensor, uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_master_transmit(sensor->dev, tx, sizeof(tx), 50);
}

static esp_err_t ina219_read_reg(ina219_sensor_t *sensor, uint8_t reg, uint16_t *value)
{
    uint8_t rx[2] = {0};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(sensor->dev, &reg, 1, rx, sizeof(rx), 50),
        TAG,
        "%s read reg 0x%02X failed",
        sensor->label,
        reg);
    *value = (uint16_t)((rx[0] << 8) | rx[1]);
    return ESP_OK;
}

static esp_err_t ina219_read_current_ma(ina219_sensor_t *sensor, float *current_ma)
{
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(ina219_read_reg(sensor, INA219_REG_CURRENT, &raw), TAG, "%s current read failed", sensor->label);
    *current_ma = (float)(int16_t)raw * INA219_CURRENT_LSB_MA;
    return ESP_OK;
}

esp_err_t ina219_init(ina219_sensor_t *sensor)
{
    ESP_RETURN_ON_ERROR(
        i2c_master_probe(i2c_bus_get(), sensor->address, 30),
        TAG,
        "%s missing at 0x%02X",
        sensor->label,
        sensor->address);
    ESP_RETURN_ON_ERROR(i2c_bus_attach_device(sensor->address, &sensor->dev), TAG, "%s attach failed", sensor->label);
    ESP_RETURN_ON_ERROR(ina219_write_reg(sensor, INA219_REG_CALIBRATION, INA219_CAL_32V_2A), TAG, "%s calibration failed", sensor->label);
    ESP_RETURN_ON_ERROR(ina219_write_reg(sensor, INA219_REG_CONFIG, INA219_CONFIG_32V_2A), TAG, "%s config failed", sensor->label);
    sensor->present = true;
    return ESP_OK;
}

bool ina219_sample_update(ina219_sensor_t *sensor, int64_t now_us)
{
    if (!sensor->present) {
        return false;
    }
    if (sensor->last_sample_us != 0 && (now_us - sensor->last_sample_us) < INA219_SAMPLE_INTERVAL_US) {
        return false;
    }

    float new_current_ma = 0.0f;
    if (ina219_read_current_ma(sensor, &new_current_ma) != ESP_OK) {
        return false;
    }

    if (sensor->has_sample) {
        int64_t dt_us = now_us - sensor->last_sample_us;
        if (dt_us > 0) {
            float dt_h = (float)dt_us / 3600000000.0f;
            sensor->total_mah += ((sensor->current_ma + new_current_ma) * 0.5f) * dt_h;
        }
    } else {
        sensor->has_sample = true;
    }

    sensor->current_ma = new_current_ma;
    sensor->last_sample_us = now_us;
    return true;
}
