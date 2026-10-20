#ifndef UI_REDRAW_POLICY_H
#define UI_REDRAW_POLICY_H

#include <stdint.h>

/* CH58x TMOS clock ticks are 625 us: background 50 ms, navigation 20 ms.
 * Measure from completed redraws so continuous UART traffic has a bounded
 * display duty cycle. All fields belong to the main loop, never an ISR. */
#define UI_BACKGROUND_REDRAW_TICKS 80u
#define UI_NAVIGATION_REDRAW_TICKS 32u

typedef struct
{
    uint32_t last_end;
    uint32_t navigation_since;
    uint8_t has_drawn;
    uint8_t navigation_pending;
} ui_redraw_policy_t;

static inline void ui_redraw_navigation(ui_redraw_policy_t *s, uint32_t now)
{
    /* Keep the oldest waiting change when multiple pages are coalesced. */
    if (!s->navigation_pending) s->navigation_since = now;
    s->navigation_pending = 1u;
}

static inline uint32_t ui_redraw_wait(const ui_redraw_policy_t *s, uint32_t now)
{
    uint32_t interval = s->navigation_pending ? UI_NAVIGATION_REDRAW_TICKS :
                                               UI_BACKGROUND_REDRAW_TICKS;
    uint32_t elapsed = (uint32_t)(now - s->last_end);
    if (!s->has_drawn || elapsed >= interval) return 0u;
    return interval - elapsed;
}

static inline void ui_redraw_complete(ui_redraw_policy_t *s, uint32_t now)
{
    s->last_end = now;
    s->has_drawn = 1u;
    s->navigation_pending = 0u;
}

static inline void ui_redraw_cancel(ui_redraw_policy_t *s)
{
    s->navigation_pending = 0u;
}

#endif
