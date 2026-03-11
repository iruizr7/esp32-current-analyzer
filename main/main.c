#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "app_config.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "framebuffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ina219.h"
#include "i2c_bus.h"
#include "max17048.h"
#include "output_format.h"
#include "ssd1306.h"

static const char *TAG = APP_LOG_TAG;

static ina219_sensor_t s_sensors[] = {
    {.address = INA219_ADDR_1, .label = INA1_PREFIX},
    {.address = INA219_ADDR_2, .label = INA2_PREFIX},
};
static max17048_gauge_t s_battery = {
    .address = MAX17048_ADDR,
};

void app_main(void)
{
    bool oled_ok = false;
    char line1[24] = {0};
    char line2[24] = {0};
    char line3[24] = {0};
    char line4[24] = {0};
    char line5[24] = {0};
    char line6_left[16] = {0};
    char line6_right[16] = {0};
    int64_t next_oled_update_us = 0;

    ESP_ERROR_CHECK(i2c_bus_init());
    i2c_bus_scan();

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
        ESP_LOGI(TAG, "SSD1306 ready at 0x%02X", ssd1306_get_address());
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
            format_value_ma(line1, sizeof(line1), s_sensors[0].has_sample, s_sensors[0].current_ma);
            format_value_mah(line2, sizeof(line2), s_sensors[0].has_sample, s_sensors[0].total_mah);
            format_value_ma(line3, sizeof(line3), s_sensors[1].has_sample, s_sensors[1].current_ma);
            format_value_mah(line4, sizeof(line4), s_sensors[1].has_sample, s_sensors[1].total_mah);
            format_line_battery(line5, sizeof(line5), s_battery.has_sample, s_battery.voltage_v, s_battery.soc_percent);
            format_line_battery_extra(
                line6_left, sizeof(line6_left),
                line6_right, sizeof(line6_right),
                s_battery.has_sample, s_battery.crate_percent_per_hour, s_battery.status_raw);

            framebuffer_clear();
            fb_draw_text(0, 0, "ina1:");
            fb_draw_text(INA_VALUE_X, 0, line1);
            fb_draw_text(INA_UNIT_X, 0, "mA");
            fb_draw_text(0, OLED_LINE_STEP * 1, "ina1T:");
            fb_draw_text(INA_VALUE_X, OLED_LINE_STEP * 1, line2);
            fb_draw_text(INA_UNIT_X, OLED_LINE_STEP * 1, "mAh");
            fb_draw_text(0, OLED_LINE_STEP * 2, "ina2:");
            fb_draw_text(INA_VALUE_X, OLED_LINE_STEP * 2, line3);
            fb_draw_text(INA_UNIT_X, OLED_LINE_STEP * 2, "mA");
            fb_draw_text(0, OLED_LINE_STEP * 3, "ina2T:");
            fb_draw_text(INA_VALUE_X, OLED_LINE_STEP * 3, line4);
            fb_draw_text(INA_UNIT_X, OLED_LINE_STEP * 3, "mAh");
            fb_draw_text(0, OLED_LINE_STEP * 4, line5);
            fb_draw_text(0, OLED_LINE_STEP * 5, line6_left);
            fb_draw_text(fb_text_right_x(line6_right), OLED_LINE_STEP * 5, line6_right);
            if (ssd1306_refresh_pages(framebuffer_data(), 0, SSD1306_PAGES - 1) != ESP_OK) {
                oled_ok = false;
                ESP_LOGW(TAG, "fallo escribiendo OLED, continuo solo por serie");
            }
        }

        if (!sampled_any) {
            taskYIELD();
        }
    }
}
