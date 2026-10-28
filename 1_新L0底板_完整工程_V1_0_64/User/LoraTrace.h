#ifndef LORA_TRACE_H
#define LORA_TRACE_H
#include <stdint.h>
#include "RadioDiag.h"

#define LORA_TRACE_BUILD "LK3_20261028"
#define LORA_TRACE_CAPACITY 32U
enum {
    LT_READY, LT_REPLACE, LT_SELF_TX, LT_FORWARD_TX, LT_TX_EXIT,
    LT_RX, LT_BAD_LEN, LT_BAD_SUM, LT_FORWARD_QUEUE, LT_FORWARD_BUSY,
    LT_SELF_TIMEOUT, LT_FORWARD_TIMEOUT, LT_FORWARD_FAIL,
    LT_SELF_OK_D, LT_SELF_OK_A, LT_SELF_OK_OD,
    LT_FORWARD_OK_D, LT_FORWARD_OK_A, LT_FORWARD_OK_OD,
    LT_HEAL_RESET, LT_CANCEL, LT_WATCHDOG, LT_CAD_RX_END,
    LT_HEAL_BEGIN, LT_HEAL_END, LT_EVENT_COUNT
};
enum {
    RF_PRE_TX, RF_TX_EXIT, RF_RX_OPEN, RF_CONFIRM_TIMEOUT,
    RF_HEAL_BEFORE, RF_HEAL_AFTER, RF_REASON_COUNT
};
typedef struct {
    uint32_t rt, cad, hit;
    uint8_t part, host, tx, rx, forward, attempt, pending;
} LoraTraceContext;

/* Main-loop only. Never prints or modifies the radio, protocol or payload. */
void LoraTrace_Record(const LoraTraceContext *ctx, uint8_t event,
                      uint32_t seq, const uint8_t *frame, uint16_t len,
                      uint16_t detail);
/* At most one line per call, only when the caller allows safe output.
 * Event rt is the capture time (100ms ticks), not the later PC print time. */
void LoraTrace_Service(const LoraTraceContext *ctx, uint8_t allow_output);
uint32_t LoraTrace_DataKey(const uint8_t *data, uint16_t len);
void LoraTrace_Radio(const LoraTraceContext *ctx, uint8_t reason,
                      uint32_t seq, const RadioDiagSnapshot *snapshot);
#endif
