#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ina219.h"

void format_value_ma(char *buf, size_t len, bool ok, float current_ma);
void format_value_mah(char *buf, size_t len, bool ok, float total_mah);
void print_teleplot_current(const ina219_sensor_t *sensor);
void print_teleplot_total(const ina219_sensor_t *sensor);
void print_teleplot_battery_voltage(bool ok, float voltage_v);
void print_teleplot_battery_soc(bool ok, float soc_percent);
void format_line_battery(char *buf, size_t len, bool ok, float voltage_v, float soc_percent);
void format_line_battery_extra(
    char *left,
    size_t left_len,
    char *right,
    size_t right_len,
    bool ok,
    float crate_percent_per_hour,
    uint16_t status_raw);
