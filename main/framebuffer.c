#include "framebuffer.h"

#include <string.h>

#include "app_config.h"

static uint8_t s_framebuffer[SSD1306_FRAMEBUFFER_SIZE];

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
    case 'C': out[0] = 0x3E; out[1] = 0x41; out[2] = 0x41; out[3] = 0x41; out[4] = 0x22; break;
    case 'E': out[0] = 0x7F; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x41; break;
    case 'H': out[0] = 0x7F; out[1] = 0x08; out[2] = 0x08; out[3] = 0x08; out[4] = 0x7F; break;
    case 'I': out[0] = 0x00; out[1] = 0x41; out[2] = 0x7F; out[3] = 0x41; out[4] = 0x00; break;
    case 'K': out[0] = 0x7F; out[1] = 0x08; out[2] = 0x14; out[3] = 0x22; out[4] = 0x41; break;
    case 'L': out[0] = 0x7F; out[1] = 0x40; out[2] = 0x40; out[3] = 0x40; out[4] = 0x40; break;
    case 'M': out[0] = 0x7F; out[1] = 0x02; out[2] = 0x04; out[3] = 0x02; out[4] = 0x7F; break;
    case 'N': out[0] = 0x7F; out[1] = 0x04; out[2] = 0x08; out[3] = 0x10; out[4] = 0x7F; break;
    case 'O': out[0] = 0x3E; out[1] = 0x41; out[2] = 0x41; out[3] = 0x41; out[4] = 0x3E; break;
    case 'R': out[0] = 0x7F; out[1] = 0x09; out[2] = 0x19; out[3] = 0x29; out[4] = 0x46; break;
    case 'S': out[0] = 0x26; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x32; break;
    case 'T': out[0] = 0x01; out[1] = 0x01; out[2] = 0x7F; out[3] = 0x01; out[4] = 0x01; break;
    case 'V': out[0] = 0x1F; out[1] = 0x20; out[2] = 0x40; out[3] = 0x20; out[4] = 0x1F; break;
    case 'a': out[0] = 0x20; out[1] = 0x54; out[2] = 0x54; out[3] = 0x54; out[4] = 0x78; break;
    case 'b': out[0] = 0x7F; out[1] = 0x48; out[2] = 0x44; out[3] = 0x44; out[4] = 0x38; break;
    case 'c': out[0] = 0x38; out[1] = 0x44; out[2] = 0x44; out[3] = 0x44; out[4] = 0x28; break;
    case 'e': out[0] = 0x38; out[1] = 0x54; out[2] = 0x54; out[3] = 0x54; out[4] = 0x18; break;
    case 'h': out[0] = 0x7F; out[1] = 0x08; out[2] = 0x04; out[3] = 0x04; out[4] = 0x78; break;
    case 'i': out[0] = 0x00; out[1] = 0x44; out[2] = 0x7D; out[3] = 0x40; out[4] = 0x00; break;
    case 'm': out[0] = 0x7C; out[1] = 0x04; out[2] = 0x18; out[3] = 0x04; out[4] = 0x78; break;
    case 'n': out[0] = 0x7C; out[1] = 0x04; out[2] = 0x04; out[3] = 0x04; out[4] = 0x78; break;
    case 'o': out[0] = 0x38; out[1] = 0x44; out[2] = 0x44; out[3] = 0x44; out[4] = 0x38; break;
    case 'r': out[0] = 0x7C; out[1] = 0x08; out[2] = 0x04; out[3] = 0x04; out[4] = 0x08; break;
    case 's': out[0] = 0x48; out[1] = 0x54; out[2] = 0x54; out[3] = 0x54; out[4] = 0x24; break;
    case 't': out[0] = 0x04; out[1] = 0x04; out[2] = 0x3F; out[3] = 0x44; out[4] = 0x24; break;
    case ':': out[0] = 0x00; out[1] = 0x36; out[2] = 0x36; out[3] = 0x00; out[4] = 0x00; break;
    case '.': out[0] = 0x00; out[1] = 0x60; out[2] = 0x60; out[3] = 0x00; out[4] = 0x00; break;
    case '%': out[0] = 0x43; out[1] = 0x33; out[2] = 0x08; out[3] = 0x66; out[4] = 0x61; break;
    case '-': out[0] = 0x08; out[1] = 0x08; out[2] = 0x08; out[3] = 0x08; out[4] = 0x08; break;
    case ' ': out[0] = 0x00; out[1] = 0x00; out[2] = 0x00; out[3] = 0x00; out[4] = 0x00; break;
    default: out[0] = 0x00; out[1] = 0x00; out[2] = 0x5F; out[3] = 0x00; out[4] = 0x00; break;
    }
}

uint8_t *framebuffer_data(void)
{
    return s_framebuffer;
}

void framebuffer_clear(void)
{
    memset(s_framebuffer, 0, sizeof(s_framebuffer));
}

void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }

    size_t idx = (size_t)x + ((size_t)y / 8) * SSD1306_WIDTH;
    uint8_t mask = (uint8_t)(1U << (y & 7));
    if (on) {
        s_framebuffer[idx] |= mask;
    } else {
        s_framebuffer[idx] &= (uint8_t)~mask;
    }
}

void fb_draw_char(int x, int y, char c)
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

void fb_draw_text(int x, int y, const char *text)
{
    for (int i = 0; text[i] != '\0'; i++) {
        fb_draw_char(x + i * 6, y, text[i]);
    }
}

int fb_text_width(const char *text)
{
    return (int)strlen(text) * 6;
}

int fb_text_right_x(const char *text)
{
    int x = SSD1306_WIDTH - fb_text_width(text);
    if (x < 0) {
        x = 0;
    }
    return x;
}
