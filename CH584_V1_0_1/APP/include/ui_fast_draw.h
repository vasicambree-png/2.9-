#ifndef UI_FAST_DRAW_H
#define UI_FAST_DRAW_H

#include "u8g2.h"

#ifdef __HIGH_CODE
#define UI_FAST_CODE __HIGH_CODE
#else
#define UI_FAST_CODE
#endif

/* Only accelerate this project's full 168x384 vertical-LSB R3 buffer.
 * Other layouts, partial buffers and partially clipped shapes keep u8g2. */
static inline uint8_t ui_fast_can_draw(const u8g2_t *s, u8g2_uint_t x,
    u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h)
{
#ifdef UI_FAST_DRAW_DISABLE
    return 0u;
#else
    if (!w || !h || !s->tile_buf_ptr || s->draw_color > 2u ||
        s->cb != U8G2_R3 || s->ll_hvline != u8g2_ll_hvline_vertical_top_lsb ||
        s->width != 384u || s->height != 168u || s->pixel_buf_width != 168u ||
        s->pixel_buf_height != 384u || s->tile_buf_height != 48u ||
        s->tile_curr_row || s->pixel_curr_row || s->buf_y0 || s->buf_y1 != 384u)
        return 0u;
#ifdef U8G2_WITH_CLIP_WINDOW_SUPPORT
    if (!s->is_page_clip_window_intersection) return 0u;
#endif
    return x >= s->user_x0 && y >= s->user_y0 &&
           (uint32_t)x + w <= 384u && (uint32_t)y + h <= 168u &&
           (uint32_t)x + w <= s->user_x1 && (uint32_t)y + h <= s->user_y1;
#endif
}

static inline uint8_t ui_fast_reverse8(uint8_t v)
{
    v = (uint8_t)(((v & 0x55u) << 1) | ((v >> 1) & 0x55u));
    v = (uint8_t)(((v & 0x33u) << 2) | ((v >> 2) & 0x33u));
    return (uint8_t)((v << 4) | (v >> 4));
}

static inline void ui_fast_bitmap_byte(uint8_t *dst, uint8_t bits,
    uint8_t mask, uint8_t color, uint8_t transparent)
{
    if (transparent)
    {
        if (color == 0u) *dst &= (uint8_t)~bits;
        else if (color == 1u) *dst |= bits;
        else *dst ^= bits;
    }
    else
    {
        uint8_t value;
        if (color == 0u) value = (uint8_t)(mask & (uint8_t)~bits);
        else if (color == 1u) value = bits;
        else value = (uint8_t)((uint8_t)~*dst & bits);
        *dst = (uint8_t)((*dst & (uint8_t)~mask) | value);
    }
}

UI_FAST_CODE
static void ui_fast_DrawXBMP(u8g2_t *s, u8g2_uint_t x, u8g2_uint_t y,
    u8g2_uint_t w, u8g2_uint_t h, const uint8_t *bitmap)
{
    uint16_t row, col, stride;
    if (!ui_fast_can_draw(s, x, y, w, h))
    {
        u8g2_DrawXBMP(s, x, y, w, h, bitmap);
        return;
    }
    stride = (uint16_t)((w + 7u) >> 3);
    for (row = 0u; row < h; row++)
    {
        for (col = 0u; col < w; col += 8u)
        {
            uint8_t n = (uint8_t)((w - col < 8u) ? w - col : 8u);
            uint16_t low = (uint16_t)(384u - x - col - n);
            uint8_t shift = (uint8_t)(low & 7u);
            uint16_t mask = (uint16_t)(((1u << n) - 1u) << shift);
            uint16_t bits = (uint16_t)((uint16_t)(ui_fast_reverse8(
                u8x8_pgm_read(bitmap + row * stride + (col >> 3))) >> (8u - n)) << shift);
            uint8_t *dst = s->tile_buf_ptr + (low >> 3) * 168u + y + row;
            ui_fast_bitmap_byte(dst, (uint8_t)bits, (uint8_t)mask,
                                s->draw_color, s->bitmap_transparency);
            if (mask >> 8)
                ui_fast_bitmap_byte(dst + 168u, (uint8_t)(bits >> 8), (uint8_t)(mask >> 8),
                                    s->draw_color, s->bitmap_transparency);
        }
    }
}

UI_FAST_CODE
static void ui_fast_DrawBox(u8g2_t *s, u8g2_uint_t x, u8g2_uint_t y,
    u8g2_uint_t w, u8g2_uint_t h)
{
    uint16_t low, high, page, col;
    if (!ui_fast_can_draw(s, x, y, w, h))
    {
        u8g2_DrawBox(s, x, y, w, h);
        return;
    }
    low = (uint16_t)(384u - x - w);
    high = (uint16_t)(384u - x);
    for (page = (uint16_t)(low >> 3); page <= (uint16_t)((high - 1u) >> 3); page++)
    {
        uint16_t base = (uint16_t)(page << 3);
        uint8_t first = (uint8_t)((low > base) ? low - base : 0u);
        uint8_t last = (uint8_t)((high < base + 8u) ? high - base : 8u);
        uint8_t mask = (uint8_t)(((1u << last) - 1u) & ~((1u << first) - 1u));
        uint8_t *dst = s->tile_buf_ptr + page * 168u + y;
        for (col = 0u; col < h; col++)
        {
            if (s->draw_color == 0u) dst[col] &= (uint8_t)~mask;
            else if (s->draw_color == 1u) dst[col] |= mask;
            else dst[col] ^= mask;
        }
    }
}

#undef UI_FAST_CODE
#endif
