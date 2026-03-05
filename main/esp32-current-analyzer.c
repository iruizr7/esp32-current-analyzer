#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_FREQ_HZ 400000

#define INA219_ADDR_1 0x40
#define INA219_ADDR_2 0x41
#define MAX17048_ADDR 0x36

#define INA219_REG_CONFIG 0x00
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05
#define INA219_CONFIG_32V_2A 0x399F
#define INA219_CAL_32V_2A 0x1000
#define INA219_CURRENT_LSB_MA 0.1f
#define INA1_PREFIX "ina1"
#define INA2_PREFIX "ina2"
#define OLED_REFRESH_INTERVAL_US 500000
#define INA219_SAMPLE_INTERVAL_US 1100
#define EMAH_EMIT_STEP 0.01f
#define MAX17048_REG_VCELL 0x02
#define MAX17048_REG_SOC 0x04
#define MAX17048_SAMPLE_INTERVAL_US 100000
#define MAX17048_RETRY_INTERVAL_US 1000000
#define MAX17048_INIT_RETRIES 8
#define MAX17048_INIT_RETRY_DELAY_MS 50

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES (SSD1306_HEIGHT / 8)
#define SSD1306_ADDR_1 0x3C
#define SSD1306_ADDR_2 0x3D

static const char *TAG = "ina219-oled";
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_oled_dev = NULL;
static uint8_t s_oled_addr = SSD1306_ADDR_1;
static uint8_t s_fb[SSD1306_WIDTH * SSD1306_PAGES];

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

typedef struct {
    uint8_t address;
    i2c_master_dev_handle_t dev;
    bool present;
    bool has_sample;
    float voltage_v;
    float soc_percent;
    int64_t last_sample_us;
    int64_t last_retry_us;
} max17048_gauge_t;

static ina219_sensor_t s_sensors[] = {
    {.address = INA219_ADDR_1, .label = INA1_PREFIX, .dev = NULL, .present = false, .has_sample = false, .current_ma = 0.0f, .total_mah = 0.0f, .last_sample_us = 0, .last_emit_current_ma = 0.0f, .last_emit_total_mah = 0.0f, .has_emitted_current = false, .has_emitted_total = false},
    {.address = INA219_ADDR_2, .label = INA2_PREFIX, .dev = NULL, .present = false, .has_sample = false, .current_ma = 0.0f, .total_mah = 0.0f, .last_sample_us = 0, .last_emit_current_ma = 0.0f, .last_emit_total_mah = 0.0f, .has_emitted_current = false, .has_emitted_total = false},
};
static max17048_gauge_t s_battery = {
    .address = MAX17048_ADDR,
    .dev = NULL,
    .present = false,
    .has_sample = false,
    .voltage_v = 0.0f,
    .soc_percent = 0.0f,
    .last_sample_us = 0,
    .last_retry_us = 0,
};

static esp_err_t max17048_read_voltage_v(max17048_gauge_t *gauge, float *voltage_v);
static esp_err_t max17048_read_soc_percent(max17048_gauge_t *gauge, float *soc_percent);

static esp_err_t i2c_master_init(void)
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

static esp_err_t i2c_attach_device(uint8_t addr, i2c_master_dev_handle_t *dev)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_i2c_bus, &cfg, dev);
}

static void i2c_scan(void)
{
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 20) == ESP_OK) {
            ESP_LOGI(TAG, "I2C device at 0x%02X", addr);
        }
    }
}

static esp_err_t ina219_write_reg(ina219_sensor_t *sensor, uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_master_transmit(sensor->dev, tx, sizeof(tx), 50);
}

static esp_err_t ina219_read_reg(ina219_sensor_t *sensor, uint8_t reg, uint16_t *value)
{
    uint8_t rx[2] = {0};
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(sensor->dev, &reg, 1, rx, sizeof(rx), 50), TAG, "%s read reg 0x%02X failed", sensor->label, reg);
    *value = (uint16_t)((rx[0] << 8) | rx[1]);
    return ESP_OK;
}

static esp_err_t ina219_init(ina219_sensor_t *sensor)
{
    ESP_RETURN_ON_ERROR(i2c_master_probe(s_i2c_bus, sensor->address, 30), TAG, "%s missing at 0x%02X", sensor->label, sensor->address);
    ESP_RETURN_ON_ERROR(i2c_attach_device(sensor->address, &sensor->dev), TAG, "%s attach failed", sensor->label);
    ESP_RETURN_ON_ERROR(ina219_write_reg(sensor, INA219_REG_CALIBRATION, INA219_CAL_32V_2A), TAG, "%s calibration failed", sensor->label);
    ESP_RETURN_ON_ERROR(ina219_write_reg(sensor, INA219_REG_CONFIG, INA219_CONFIG_32V_2A), TAG, "%s config failed", sensor->label);
    sensor->present = true;
    return ESP_OK;
}

static esp_err_t ina219_read_current_ma(ina219_sensor_t *sensor, float *current_ma)
{
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(ina219_read_reg(sensor, INA219_REG_CURRENT, &raw), TAG, "%s current read failed", sensor->label);
    *current_ma = (float)(int16_t)raw * INA219_CURRENT_LSB_MA;
    return ESP_OK;
}

static bool ina219_sample_update(ina219_sensor_t *sensor, int64_t now_us)
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

static esp_err_t max17048_read_reg(max17048_gauge_t *gauge, uint8_t reg, uint16_t *value)
{
    uint8_t rx[2] = {0};
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(gauge->dev, &reg, 1, rx, sizeof(rx), 50), TAG, "max17048 read reg 0x%02X failed", reg);
    *value = (uint16_t)((rx[0] << 8) | rx[1]);
    return ESP_OK;
}

static esp_err_t max17048_init(max17048_gauge_t *gauge)
{
    if (gauge->dev != NULL) {
        i2c_master_bus_rm_device(gauge->dev);
        gauge->dev = NULL;
    }

    ESP_RETURN_ON_ERROR(i2c_attach_device(gauge->address, &gauge->dev), TAG, "max17048 attach failed");

    for (int i = 0; i < MAX17048_INIT_RETRIES; i++) {
        float voltage_v = 0.0f;
        float soc_percent = 0.0f;
        if (max17048_read_voltage_v(gauge, &voltage_v) == ESP_OK &&
            max17048_read_soc_percent(gauge, &soc_percent) == ESP_OK) {
            gauge->voltage_v = voltage_v;
            gauge->soc_percent = soc_percent;
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

static bool max17048_sample_update(max17048_gauge_t *gauge, int64_t now_us)
{
    if (!gauge->present) {
        return false;
    }
    if (gauge->last_sample_us != 0 && (now_us - gauge->last_sample_us) < MAX17048_SAMPLE_INTERVAL_US) {
        return false;
    }

    float voltage_v = 0.0f;
    float soc_percent = 0.0f;
    if (max17048_read_voltage_v(gauge, &voltage_v) != ESP_OK) {
        return false;
    }
    if (max17048_read_soc_percent(gauge, &soc_percent) != ESP_OK) {
        return false;
    }

    gauge->voltage_v = voltage_v;
    gauge->soc_percent = soc_percent;
    gauge->has_sample = true;
    gauge->last_sample_us = now_us;
    return true;
}

static void max17048_retry_if_needed(max17048_gauge_t *gauge, int64_t now_us)
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

static esp_err_t ssd1306_detect_and_init(void)
{
    static const uint8_t probe[] = {SSD1306_ADDR_1, SSD1306_ADDR_2};
    static const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF,
    };

    for (size_t i = 0; i < ARRAY_SIZE(probe); i++) {
        if (i2c_master_probe(s_i2c_bus, probe[i], 30) != ESP_OK) {
            continue;
        }
        ESP_RETURN_ON_ERROR(i2c_attach_device(probe[i], &s_oled_dev), TAG, "oled attach failed");
        s_oled_addr = probe[i];

        for (size_t k = 0; k < ARRAY_SIZE(init_cmds); k++) {
            ESP_RETURN_ON_ERROR(ssd1306_send_cmd(init_cmds[k]), TAG, "oled init cmd failed");
        }
        memset(s_fb, 0, sizeof(s_fb));
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t ssd1306_refresh_pages(uint8_t first_page, uint8_t last_page)
{
    if (last_page >= SSD1306_PAGES || first_page > last_page) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t page = first_page; page <= last_page; page++) {
        ESP_RETURN_ON_ERROR(ssd1306_send_cmd(0xB0 | page), TAG, "set page failed");
        ESP_RETURN_ON_ERROR(ssd1306_send_cmd(0x00), TAG, "set col low failed");
        ESP_RETURN_ON_ERROR(ssd1306_send_cmd(0x10), TAG, "set col high failed");
        ESP_RETURN_ON_ERROR(ssd1306_send_data(&s_fb[page * SSD1306_WIDTH], SSD1306_WIDTH), TAG, "page write failed");
    }
    return ESP_OK;
}

static void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }
    size_t idx = (size_t)x + ((size_t)y / 8) * SSD1306_WIDTH;
    uint8_t mask = (uint8_t)(1U << (y & 7));
    if (on) {
        s_fb[idx] |= mask;
    } else {
        s_fb[idx] &= (uint8_t)~mask;
    }
}

static void font5x7_get(char c, uint8_t out[5])
{
    switch (c) {
    case '0': out[0] = 0x3E; out[1] = 0x51; out[2] = 0x49; out[3] = 0x45; out[4] = 0x3E; break;
    case '1': out[0] = 0x00; out[1] = 0x42; out[2] = 0x7F; out[3] = 0x40; out[4] = 0x00; break;
    case '2': out[0] = 0x62; out[1] = 0x51; out[2] = 0x49; out[3] = 0x49; out[4] = 0x46; break;
    case '3': out[0] = 0x22; out[1] = 0x41; out[2] = 0x49; out[3] = 0x49; out[4] = 0x36; break;
    case '4': out[0] = 0x18; out[1] = 0x14; out[2] = 0x12; out[3] = 0x7F; out[4] = 0x10; break;
    case '5': out[0] = 0x2F; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x31; break;
    case '6': out[0] = 0x3E; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x32; break;
    case '7': out[0] = 0x01; out[1] = 0x71; out[2] = 0x09; out[3] = 0x05; out[4] = 0x03; break;
    case '8': out[0] = 0x36; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x36; break;
    case '9': out[0] = 0x26; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x3E; break;
    case 'A': out[0] = 0x7E; out[1] = 0x11; out[2] = 0x11; out[3] = 0x11; out[4] = 0x7E; break;
    case 'E': out[0] = 0x7F; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x41; break;
    case 'I': out[0] = 0x00; out[1] = 0x41; out[2] = 0x7F; out[3] = 0x41; out[4] = 0x00; break;
    case 'M': out[0] = 0x7F; out[1] = 0x02; out[2] = 0x04; out[3] = 0x02; out[4] = 0x7F; break;
    case 'N': out[0] = 0x7F; out[1] = 0x04; out[2] = 0x08; out[3] = 0x10; out[4] = 0x7F; break;
    case 'R': out[0] = 0x7F; out[1] = 0x09; out[2] = 0x19; out[3] = 0x29; out[4] = 0x46; break;
    case 'T': out[0] = 0x01; out[1] = 0x01; out[2] = 0x7F; out[3] = 0x01; out[4] = 0x01; break;
    case 'V': out[0] = 0x1F; out[1] = 0x20; out[2] = 0x40; out[3] = 0x20; out[4] = 0x1F; break;
    case 'a': out[0] = 0x20; out[1] = 0x54; out[2] = 0x54; out[3] = 0x54; out[4] = 0x78; break;
    case 'b': out[0] = 0x7F; out[1] = 0x48; out[2] = 0x44; out[3] = 0x44; out[4] = 0x38; break;
    case 'c': out[0] = 0x38; out[1] = 0x44; out[2] = 0x44; out[3] = 0x44; out[4] = 0x28; break;
    case 'h': out[0] = 0x7F; out[1] = 0x08; out[2] = 0x04; out[3] = 0x04; out[4] = 0x78; break;
    case 'i': out[0] = 0x00; out[1] = 0x44; out[2] = 0x7D; out[3] = 0x40; out[4] = 0x00; break;
    case 'm': out[0] = 0x7C; out[1] = 0x04; out[2] = 0x18; out[3] = 0x04; out[4] = 0x78; break;
    case 'n': out[0] = 0x7C; out[1] = 0x04; out[2] = 0x04; out[3] = 0x04; out[4] = 0x78; break;
    case 'o': out[0] = 0x38; out[1] = 0x44; out[2] = 0x44; out[3] = 0x44; out[4] = 0x38; break;
    case 's': out[0] = 0x48; out[1] = 0x54; out[2] = 0x54; out[3] = 0x54; out[4] = 0x24; break;
    case 't': out[0] = 0x04; out[1] = 0x04; out[2] = 0x3F; out[3] = 0x44; out[4] = 0x24; break;
    case ':': out[0] = 0x00; out[1] = 0x36; out[2] = 0x36; out[3] = 0x00; out[4] = 0x00; break;
    case '.': out[0] = 0x00; out[1] = 0x60; out[2] = 0x60; out[3] = 0x00; out[4] = 0x00; break;
    case '%': out[0] = 0x43; out[1] = 0x33; out[2] = 0x08; out[3] = 0x66; out[4] = 0x61; break;
    case '-': out[0] = 0x08; out[1] = 0x08; out[2] = 0x08; out[3] = 0x08; out[4] = 0x08; break;
    case ' ': out[0] = 0x00; out[1] = 0x00; out[2] = 0x00; out[3] = 0x00; out[4] = 0x00; break;
    default:  out[0] = 0x00; out[1] = 0x00; out[2] = 0x5F; out[3] = 0x00; out[4] = 0x00; break;
    }
}

static void fb_draw_char(int x, int y, char c)
{
    uint8_t glyph[5] = {0};
    font5x7_get(c, glyph);
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1U << row)) {
                fb_set_pixel(x + col, y + row, true);
            }
        }
    }
}

static void fb_draw_text(int x, int y, const char *text)
{
    for (int i = 0; text[i] != '\0'; i++) {
        fb_draw_char(x + i * 6, y, text[i]);
    }
}

static void format_line_ma(char *buf, size_t len, const char *label, bool ok, float current_ma)
{
    if (ok) {
        snprintf(buf, len, "%s:%8.2f mA", label, current_ma);
    } else {
        snprintf(buf, len, "%s:    ERR", label);
    }
}

static void format_line_mah(char *buf, size_t len, const char *label, bool ok, float total_mah)
{
    if (ok) {
        snprintf(buf, len, "%sT:%8.2f mAh", label, total_mah);
    } else {
        snprintf(buf, len, "%sT:   ERR", label);
    }
}

static void print_teleplot_current(const ina219_sensor_t *sensor)
{
    if (sensor->has_sample) {
        printf(">%s:%.2f\n", sensor->label, sensor->current_ma);
    } else {
        printf(">%s:nan\n", sensor->label);
    }
}

static void print_teleplot_total(const ina219_sensor_t *sensor)
{
    if (sensor->has_sample) {
        printf(">%sT:%.2f\n", sensor->label, sensor->total_mah);
    } else {
        printf(">%sT:nan\n", sensor->label);
    }
}

static void format_line_battery(char *buf, size_t len, bool ok, float voltage_v, float soc_percent)
{
    if (ok) {
        snprintf(buf, len, "bat:%.2fV soc:%.1f%%", voltage_v, soc_percent);
    } else {
        snprintf(buf, len, "bat: ERR soc: ERR");
    }
}

void app_main(void)
{
    bool oled_ok = false;
    char line1[24] = {0};
    char line2[24] = {0};
    char line3[24] = {0};
    char line4[24] = {0};
    char line5[24] = {0};
    int64_t next_oled_update_us = 0;

    ESP_ERROR_CHECK(i2c_master_init());
    i2c_scan();

    for (size_t i = 0; i < ARRAY_SIZE(s_sensors); i++) {
        if (ina219_init(&s_sensors[i]) == ESP_OK) {
            ESP_LOGI(TAG, "%s ready at 0x%02X", s_sensors[i].label, s_sensors[i].address);
        } else {
            ESP_LOGW(TAG, "%s missing at 0x%02X", s_sensors[i].label, s_sensors[i].address);
        }
    }
    if (max17048_init(&s_battery) == ESP_OK) {
        ESP_LOGI(TAG, "max17048 ready at 0x%02X", s_battery.address);
    } else {
        ESP_LOGW(TAG, "max17048 missing at 0x%02X", s_battery.address);
    }

    if (ssd1306_detect_and_init() == ESP_OK) {
        oled_ok = true;
        ESP_LOGI(TAG, "SSD1306 ready at 0x%02X", s_oled_addr);
    } else {
        ESP_LOGW(TAG, "SSD1306 no detectada, continuo solo por serie");
    }

    while (true) {
        int64_t now = esp_timer_get_time();
        max17048_retry_if_needed(&s_battery, now);

        bool sampled_any = false;
        bool emit1_current = false;
        bool emit1_total = false;
        bool emit2_current = false;
        bool emit2_total = false;

        bool s1 = ina219_sample_update(&s_sensors[0], now);
        bool s2 = ina219_sample_update(&s_sensors[1], now);
        bool sb = max17048_sample_update(&s_battery, now);
        sampled_any = s1 || s2 || sb;

        if (s1) {
            if (!s_sensors[0].has_emitted_current ||
                fabsf(s_sensors[0].current_ma - s_sensors[0].last_emit_current_ma) > 0.0f) {
                emit1_current = true;
            }
            emit1_total = true;
        }

        if (s2) {
            if (!s_sensors[1].has_emitted_current ||
                fabsf(s_sensors[1].current_ma - s_sensors[1].last_emit_current_ma) > 0.0f) {
                emit2_current = true;
            }
            emit2_total = true;
        }

        if (emit1_current) {
            print_teleplot_current(&s_sensors[0]);
            s_sensors[0].last_emit_current_ma = s_sensors[0].current_ma;
            s_sensors[0].has_emitted_current = true;
        }
        if (emit1_total) {
            print_teleplot_total(&s_sensors[0]);
            s_sensors[0].last_emit_total_mah = s_sensors[0].total_mah;
            s_sensors[0].has_emitted_total = true;
        }
        if (emit2_current) {
            print_teleplot_current(&s_sensors[1]);
            s_sensors[1].last_emit_current_ma = s_sensors[1].current_ma;
            s_sensors[1].has_emitted_current = true;
        }
        if (emit2_total) {
            print_teleplot_total(&s_sensors[1]);
            s_sensors[1].last_emit_total_mah = s_sensors[1].total_mah;
            s_sensors[1].has_emitted_total = true;
        }

        if (oled_ok && now >= next_oled_update_us) {
            next_oled_update_us = now + OLED_REFRESH_INTERVAL_US;
            format_line_ma(line1, sizeof(line1), INA1_PREFIX, s_sensors[0].has_sample, s_sensors[0].current_ma);
            format_line_mah(line2, sizeof(line2), INA1_PREFIX, s_sensors[0].has_sample, s_sensors[0].total_mah);
            format_line_ma(line3, sizeof(line3), INA2_PREFIX, s_sensors[1].has_sample, s_sensors[1].current_ma);
            format_line_mah(line4, sizeof(line4), INA2_PREFIX, s_sensors[1].has_sample, s_sensors[1].total_mah);
            format_line_battery(line5, sizeof(line5), s_battery.has_sample, s_battery.voltage_v, s_battery.soc_percent);

            memset(s_fb, 0, sizeof(s_fb));
            fb_draw_text(0, 0, line1);
            fb_draw_text(0, 14, line2);
            fb_draw_text(0, 28, line3);
            fb_draw_text(0, 42, line4);
            fb_draw_text(0, 56, line5);
            if (ssd1306_refresh_pages(0, SSD1306_PAGES - 1) != ESP_OK) {
                oled_ok = false;
                ESP_LOGW(TAG, "fallo escribiendo OLED, continuo solo por serie");
            }
        }

        if (!sampled_any) {
            taskYIELD();
        }
    }
}
