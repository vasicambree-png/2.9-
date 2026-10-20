#ifndef SCREEN_POWER_H
#define SCREEN_POWER_H

#include <stdint.h>

/* TMOS_GetSystemClock(): one tick = 0.625 ms. No GPIO or SPI in this policy. */
#define SCREEN_TICKS_PER_SEC 1600u

typedef struct
{
    uint32_t last_activity;
    uint16_t timeout_sec;
    uint16_t page;
    uint8_t rank;
    uint8_t menu;
    uint8_t focus;
    uint8_t subpage;
    uint8_t seen;
    uint8_t expired;
} screen_power_t;

/* Only accepted navigation/settings frames call this. Live measurements are
 * deliberately absent, so periodic UART/BLE data cannot keep the display on. */
static inline void screen_power_note_page(screen_power_t *s, uint32_t now,
                                         uint8_t rank, uint8_t menu, uint8_t focus,
                                         uint16_t page, uint8_t subpage,
                                         uint16_t timeout_sec)
{
    if (!s->seen || s->rank != rank || s->menu != menu || s->focus != focus ||
        s->page != page || s->subpage != subpage || s->timeout_sec != timeout_sec)
    {
        s->last_activity = now;
        s->expired = 0u;
    }
    s->rank = rank;
    s->menu = menu;
    s->focus = focus;
    s->page = page;
    s->subpage = subpage;
    s->timeout_sec = timeout_sec;
    s->seen = 1u;
}

static inline uint8_t screen_power_is_off(screen_power_t *s, uint32_t now)
{
    /* Background power/read cycles must not turn the display on. */
    if (!s->seen) return 1u;
    /* Keep third-level selection visible until a normal page exits rank 3.
     * note_page() restarts the configured timeout on that rank change. */
    if (s->rank == 3u) return 0u;
    if (s->timeout_sec == 0u) return 0u; /* Existing setting: 0 = always on. */
    if ((uint32_t)(now - s->last_activity) >=
        (uint32_t)s->timeout_sec * SCREEN_TICKS_PER_SEC)
        s->expired = 1u;
    /* Stay off even after a full clock cycle; only actual activity clears it. */
    return s->expired;
}

/* Visible messages or an explicit page takeover can refresh activity. */
static inline void screen_power_wake(screen_power_t *s, uint32_t now)
{
    s->last_activity = now;
    s->seen = 1u;
    s->expired = 0u;
}

#endif
