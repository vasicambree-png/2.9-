#ifndef UI_LATENCY_DIAG_H
#define UI_LATENCY_DIAG_H

#include <stdint.h>

#ifndef UI_LATENCY_DIAG
#define UI_LATENCY_DIAG 0
#endif

#if UI_LATENCY_DIAG
/* Main-loop wall times for the latest UI_Control frame, in 625 us ticks.
 * No printing inside the rendering or transfer path. */
extern uint32_t ui_profile_render_ticks;
extern uint32_t ui_profile_transfer_ticks;
#endif

#endif
