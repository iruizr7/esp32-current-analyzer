#pragma once

#include <stdbool.h>
#include <stdint.h>

uint8_t *framebuffer_data(void);
void framebuffer_clear(void);
void fb_set_pixel(int x, int y, bool on);
void fb_draw_char(int x, int y, char c);
void fb_draw_text(int x, int y, const char *text);
int fb_text_width(const char *text);
int fb_text_right_x(const char *text);
