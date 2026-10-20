#ifndef SCREEN_STARTUP_H
#define SCREEN_STARTUP_H

#include <stdint.h>

enum { SCREEN_TARGET_NONE, SCREEN_TARGET_TEST, SCREEN_TARGET_PAGE, SCREEN_TARGET_MSG };

typedef struct
{
    uint8_t test, valid_page, intro_pending, resume_intro_armed;
} screen_startup_t;

/* S2 hardware reset is an additional local artwork request. Background power
 * alone stays dark; the controller's rank-5 startup command can also request
 * artwork. Never write reset/debug options here. */
static inline void screen_startup_init(screen_startup_t *s,
                                      uint8_t reset_enabled, uint8_t external_reset)
{
    s->valid_page = 0u;
    s->resume_intro_armed = 0u;
    s->test = reset_enabled && external_reset;
    s->intro_pending = s->test;
}

static inline void screen_startup_request_artwork(screen_startup_t *s)
{
    s->test = s->intro_pending = 1u;
    s->resume_intro_armed = 0u;
}

static inline void screen_startup_artwork_sent(screen_startup_t *s)
{
    s->intro_pending = 0u;
}

static inline uint8_t screen_startup_target(const screen_startup_t *s, uint8_t msg)
{
    if (msg) return SCREEN_TARGET_MSG;
    /* Clearing a shutdown/boot prompt is not yet a normal-page request.
     * Wait for that page instead of briefly exposing the old cached home. */
    if (s->resume_intro_armed) return SCREEN_TARGET_NONE;
    /* A page received in the same UART burst is cached, but cannot swallow
     * the requested first artwork transfer. It takes over on the next draw. */
    if (s->intro_pending || s->test) return SCREEN_TARGET_TEST;
    return s->valid_page ? SCREEN_TARGET_PAGE : SCREEN_TARGET_NONE;
}

#endif
