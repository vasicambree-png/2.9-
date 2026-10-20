#include <assert.h>
#include <stdio.h>
#include "ui_redraw_policy.h"

int main(void)
{
    ui_redraw_policy_t s = {0};
    /* A first page must not wait for the 500 ms BLE IO startup timer. */
    assert(ui_redraw_wait(&s, 0u) == 0u);
    ui_redraw_complete(&s, 100u);
    assert(ui_redraw_wait(&s, 110u) == 70u);
    ui_redraw_navigation(&s, 110u);
    assert(ui_redraw_wait(&s, 110u) == 22u);
    assert(ui_redraw_wait(&s, 132u) == 0u);

    /* Repeated frames preserve the oldest request and the traffic limit. */
    ui_redraw_navigation(&s, 120u);
    assert(s.navigation_since == 110u);
    assert(ui_redraw_wait(&s, 120u) == 12u);
    ui_redraw_complete(&s, 140u);
    assert(!s.navigation_pending);
    assert(ui_redraw_wait(&s, 141u) == 79u);
    ui_redraw_navigation(&s, 141u);
    assert(ui_redraw_wait(&s, 141u) == 31u);

    /* A delayed task counts elapsed clock time, not executed heartbeats. */
    assert(ui_redraw_wait(&s, 300u) == 0u);
    ui_redraw_cancel(&s);
    assert(!s.navigation_pending);
    assert(ui_redraw_wait(&s, 141u) == 79u);

    /* Short deadlines also work across a uint32 clock rollover. */
    ui_redraw_complete(&s, UINT32_MAX - 10u);
    assert(ui_redraw_wait(&s, 5u) == 64u);
    ui_redraw_navigation(&s, 5u);
    assert(ui_redraw_wait(&s, 5u) == 16u);
    assert(ui_redraw_wait(&s, 21u) == 0u);

    puts("PASS: first page, navigation priority, coalescing, traffic limit, delayed tasks, cancel, rollover");
    return 0;
}
