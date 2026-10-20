#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "ui_fast_draw.h"

#define FRAME_BYTES 8064u
static unsigned char reference[FRAME_BYTES + 32], actual[FRAME_BYTES + 32];
static unsigned char bitmap[FRAME_BYTES];
static u8g2_t stock, fast;
static u8x8_display_info_t info;
static unsigned rng = 1234567u;
static unsigned checks;

static unsigned random_value(void)
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
}

static void setup(const u8g2_cb_t *rotation, unsigned rows)
{
    memset(&stock, 0, sizeof(stock)); memset(&fast, 0, sizeof(fast));
    memset(&info, 0, sizeof(info));
    info.pixel_width = 168; info.pixel_height = 384;
    info.tile_width = 21; info.tile_height = 48;
    stock.u8x8.display_info = &info; fast.u8x8.display_info = &info;
    u8g2_SetupBuffer(&stock, reference + 16, (uint8_t)rows,
                    u8g2_ll_hvline_vertical_top_lsb, rotation);
    u8g2_SetupBuffer(&fast, actual + 16, (uint8_t)rows,
                    u8g2_ll_hvline_vertical_top_lsb, rotation);
}

static void compare(unsigned x, unsigned y, unsigned w, unsigned h,
                    unsigned color, unsigned mode, unsigned shape)
{
    unsigned i;
    memset(reference, 0xcd, sizeof(reference));
    for (i = 0; i < FRAME_BYTES; ++i) reference[i + 16] = (unsigned char)random_value();
    memcpy(actual, reference, sizeof(actual));
    for (i = 0; i < sizeof(bitmap); ++i) bitmap[i] = (unsigned char)random_value();
    u8g2_SetDrawColor(&stock, (uint8_t)color); u8g2_SetDrawColor(&fast, (uint8_t)color);
    u8g2_SetBitmapMode(&stock, (uint8_t)mode); u8g2_SetBitmapMode(&fast, (uint8_t)mode);
    if (shape == 0u)
    {
        u8g2_DrawXBMP(&stock, (uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h, bitmap);
        ui_fast_DrawXBMP(&fast, (uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h, bitmap);
    }
    else
    {
        u8g2_DrawBox(&stock, (uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h);
        ui_fast_DrawBox(&fast, (uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h);
    }
    if (memcmp(reference, actual, sizeof(reference)))
    {
        fprintf(stderr, "Mismatch: x=%u y=%u w=%u h=%u color=%u mode=%u shape=%u\n",
                x, y, w, h, color, mode, shape);
        assert(0);
    }
    for (i = 0; i < 16; i++)
    {
        assert(actual[i] == 0xcd && actual[FRAME_BYTES + 16 + i] == 0xcd);
        assert(reference[i] == 0xcd && reference[FRAME_BYTES + 16 + i] == 0xcd);
    }
    assert(fast.draw_color == color && fast.bitmap_transparency == mode);
    checks++;
}

int main(void)
{
    unsigned n, color, mode, shape;
    static const unsigned widths[] = {0, 1, 7, 8, 9, 13, 29, 47, 383, 384};
    static const unsigned xs[] = {0, 1, 7, 8, 9, 377, 383, 384, 65535};
    unsigned wi, xi;
    double old_seconds, new_seconds;
    clock_t start;
    setup(U8G2_R3, 48);
    assert(ui_fast_can_draw(&fast, 0, 0, 384, 168));
    for (wi = 0; wi < sizeof(widths)/sizeof(widths[0]); wi++)
        for (xi = 0; xi < sizeof(xs)/sizeof(xs[0]); xi++)
            for (color = 0; color < 3; color++)
                for (mode = 0; mode < 3; mode++)
                    for (shape = 0; shape < 2; shape++)
                        compare(xs[xi], 149, widths[wi], 19, color, mode, shape);
    for (n = 0; n < 6000; ++n)
        compare(random_value() % 385, random_value() % 169,
                random_value() % 385, random_value() % 169,
                random_value() % 3, random_value() % 3, n & 1u);
    u8g2_SetClipWindow(&stock, 3, 4, 370, 160);
    u8g2_SetClipWindow(&fast, 3, 4, 370, 160);
    for (n = 0; n < 800; ++n)
        compare(random_value() % 385, random_value() % 169, 29, 19, n % 3, n % 2, n & 1u);
    setup(U8G2_R0, 48);
    assert(!ui_fast_can_draw(&fast, 0, 0, 10, 10));
    for (n = 0; n < 100; ++n) compare(n, n, 29, 19, n % 3, n % 2, n & 1u);
    setup(U8G2_R3, 1);
    assert(!ui_fast_can_draw(&fast, 0, 0, 10, 10));
    for (n = 0; n < 100; ++n) compare(n, n, 29, 19, n % 3, n % 2, n & 1u);

    setup(U8G2_R3, 48);
    u8g2_SetDrawColor(&stock, 2); u8g2_SetDrawColor(&fast, 2);
    start = clock();
    for (n = 0; n < 5000; ++n) u8g2_DrawBox(&stock, 113, 1, 266, 166);
    old_seconds = (double)(clock() - start) / CLOCKS_PER_SEC;
    start = clock();
    for (n = 0; n < 5000; ++n) ui_fast_DrawBox(&fast, 113, 1, 266, 166);
    new_seconds = (double)(clock() - start) / CLOCKS_PER_SEC;
    printf("PASS: %u pixel comparisons, clipping/fallback, guard bytes and drawing state\n", checks);
    printf("Host box benchmark: stock=%.3fs fast=%.3fs (host timing only)\n", old_seconds, new_seconds);
    return 0;
}
