#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

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
static const char *QUICKSTART_CMD = "quickstart";

#define SERIAL_COMMAND_MAX_LEN 32

static ina219_sensor_t s_sensors[] = {
    {.address = INA219_ADDR_1, .label = INA1_PREFIX},
    {.address = INA219_ADDR_2, .label = INA2_PREFIX},
};
static max17048_gauge_t s_battery = {
    .address = MAX17048_ADDR,
};

static void serial_command_init(void);
static void handle_serial_commands(max17048_gauge_t *battery);
static void process_serial_command(const char *command, max17048_gauge_t *battery);

static void serial_command_init(void)
{
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags < 0) {
        ESP_LOGW(TAG, "no puedo leer flags de stdin, errno=%d", errno);
        return;
    }
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGW(TAG, "no puedo poner stdin en no bloqueante, errno=%d", errno);
        return;
    }

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void process_serial_command(const char *command, max17048_gauge_t *battery)
{
    if (strcmp(command, QUICKSTART_CMD) == 0) {
        printf("quickstart: recibido, enviando al MAX17048\r\n");
        esp_err_t err = max17048_quickstart(battery);
        if (err == ESP_OK) {
            printf("quickstart: ok\r\n");
        } else {
            printf("quickstart: error %s\r\n", esp_err_to_name(err));
        }
        return;
    }

    printf("comando desconocido: %s\r\n", command);
}

static void handle_serial_commands(max17048_gauge_t *battery)
{
    static char command[SERIAL_COMMAND_MAX_LEN];
    static size_t command_len = 0;
    static bool discarding = false;

    while (true) {
        char ch = '\0';
        ssize_t read_len = read(STDIN_FILENO, &ch, 1);
        if (read_len == 0) {
            return;
        }
        if (read_len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            ESP_LOGW(TAG, "fallo leyendo stdin, errno=%d", errno);
            return;
        }

        if (ch == '\r' || ch == '\n') {
            if (discarding) {
                discarding = false;
                command_len = 0;
                continue;
            }
            if (command_len == 0) {
                continue;
            }
            command[command_len] = '\0';
            process_serial_command(command, battery);
            command_len = 0;
            continue;
        }

        if (discarding) {
            continue;
        }
        if ((command_len + 1) >= sizeof(command)) {
            printf("comando demasiado largo\r\n");
            command_len = 0;
            discarding = true;
            continue;
        }

        command[command_len++] = ch;
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
    char line6_left[16] = {0};
    char line6_right[16] = {0};
    int64_t next_oled_update_us = 0;
    int64_t next_teleplot_report_us = 0;

    serial_command_init();

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
        handle_serial_commands(&s_battery);
        max17048_retry_if_needed(&s_battery, now);

        bool sampled_any = false;

        bool s1 = ina219_sample_update(&s_sensors[0], now);
        bool s2 = ina219_sample_update(&s_sensors[1], now);
        bool sb = max17048_sample_update(&s_battery, now);
        sampled_any = s1 || s2 || sb;

        if (next_teleplot_report_us == 0) {
            next_teleplot_report_us = now + TELEPLOT_REPORT_INTERVAL_US;
        } else if (now >= next_teleplot_report_us) {
            print_teleplot_current(&s_sensors[0]);
            print_teleplot_total(&s_sensors[0]);
            print_teleplot_current(&s_sensors[1]);
            print_teleplot_total(&s_sensors[1]);
            print_teleplot_battery_voltage(s_battery.has_sample, s_battery.voltage_v);
            print_teleplot_battery_soc(s_battery.has_sample, s_battery.soc_percent);
            next_teleplot_report_us = now + TELEPLOT_REPORT_INTERVAL_US;
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
