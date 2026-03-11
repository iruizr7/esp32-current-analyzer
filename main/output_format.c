#include "output_format.h"

#include <stdio.h>

#include "max17048.h"

static const char *max17048_status_abbrev(uint16_t status_raw)
{
    uint8_t flags = (uint8_t)(status_raw & 0x3F);

    if (flags & MAX17048_STATUS_RI) {
        return "RI";
    }
    if (flags & MAX17048_STATUS_VR) {
        return "VR";
    }
    if (flags & MAX17048_STATUS_VL) {
        return "VL";
    }
    if (flags & MAX17048_STATUS_VH) {
        return "VH";
    }
    if (flags & MAX17048_STATUS_SL) {
        return "SL";
    }
    if (flags & MAX17048_STATUS_SC) {
        return "SC";
    }
    return "OK";
}

void format_value_ma(char *buf, size_t len, bool ok, float current_ma)
{
    if (ok) {
        snprintf(buf, len, "%8.2f", current_ma);
    } else {
        snprintf(buf, len, "    ERR");
    }
}

void format_value_mah(char *buf, size_t len, bool ok, float total_mah)
{
    if (ok) {
        snprintf(buf, len, "%8.2f", total_mah);
    } else {
        snprintf(buf, len, "   ERR");
    }
}

void print_teleplot_current(const ina219_sensor_t *sensor)
{
    if (sensor->has_sample) {
        printf(">%s:%.2f\n", sensor->label, sensor->current_ma);
    } else {
        printf(">%s:nan\n", sensor->label);
    }
}

void print_teleplot_total(const ina219_sensor_t *sensor)
{
    if (sensor->has_sample) {
        printf(">%sT:%.2f\n", sensor->label, sensor->total_mah);
    } else {
        printf(">%sT:nan\n", sensor->label);
    }
}

void format_line_battery(char *buf, size_t len, bool ok, float voltage_v, float soc_percent)
{
    if (ok) {
        snprintf(buf, len, "bat:%.2fV soc:%.1f%%", voltage_v, soc_percent);
    } else {
        snprintf(buf, len, "bat: ERR soc: ERR");
    }
}

void format_line_battery_extra(
    char *left,
    size_t left_len,
    char *right,
    size_t right_len,
    bool ok,
    float crate_percent_per_hour,
    uint16_t status_raw)
{
    if (ok) {
        snprintf(left, left_len, "crate:%6.2f", crate_percent_per_hour);
        snprintf(right, right_len, "st:%s", max17048_status_abbrev(status_raw));
    } else {
        snprintf(left, left_len, "crate: ERR");
        snprintf(right, right_len, "st: ERR");
    }
}
