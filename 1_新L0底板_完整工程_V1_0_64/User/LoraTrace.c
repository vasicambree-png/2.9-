#include "LoraTrace.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t rt, seq, key, number;
    uint16_t detail, value;
    uint8_t event, part, host, src, dst, origin, cmd, len;
    uint8_t tx, rx, forward, attempt;
    uint8_t src_part, dst_part, origin_part, value_type;
} TraceEvent;
static TraceEvent s_events[LORA_TRACE_CAPACITY];
static TraceEvent s_last_tx;
static uint32_t s_count[LT_EVENT_COUNT];
static uint32_t s_lost, s_last_status, s_number;
static uint8_t s_head, s_size, s_started, s_status_due;
static uint32_t s_compact, s_repeat_seen;
static uint8_t s_counts_due;
#define TRACE_KEY_RESERVE 8U
static const char * const s_names[LT_EVENT_COUNT] = {
    "READY", "REPLACE", "SELF_TX", "FWD_TX", "TX_EXIT",
    "RX", "BAD_LEN", "BAD_SUM", "FWD_QUEUE", "FWD_BUSY",
    "SELF_TMO", "FWD_TMO", "FWD_FAIL", "SELF_OK_D", "SELF_OK_A",
    "SELF_OK_OD", "FWD_OK_D", "FWD_OK_A", "FWD_OK_OD",
    "HEAL_RESET", "CANCEL", "WATCHDOG", "CAD_RX_END", "HEAL_BEGIN", "HEAL_END"
};

#define RF_TRACE_CAPACITY 6U
typedef struct {
    RadioDiagSnapshot snapshot;
    uint32_t seq, number;
    uint8_t reason, part, host, tx, rx;
} RadioTraceEvent;
static RadioTraceEvent s_radio[RF_TRACE_CAPACITY];
static uint32_t s_radio_number, s_radio_lost;
static uint8_t s_radio_head, s_radio_size;
static uint32_t s_radio_compact, s_radio_seen_seq, s_radio_first_timeout;
static uint8_t s_radio_seen;
#define RF_KEY_RESERVE 2U
static const char * const s_radio_names[RF_REASON_COUNT] = {
    "PRE_TX", "TX_EXIT", "RX_OPEN", "CONF_TMO", "HEAL_BEFORE", "HEAL_AFTER"
};

/* Fixed queues keep recovery/completion evidence ahead of repeated attempts.
 * Bounded critical-only overload is still reported as loss, never hidden. */
static uint8_t trace_is_key(uint8_t event)
{
    return (uint8_t)(event == LT_SELF_TIMEOUT || event == LT_FORWARD_FAIL ||
        (event >= LT_SELF_OK_D && event <= LT_FORWARD_OK_OD) ||
        event == LT_WATCHDOG || event == LT_HEAL_BEGIN || event == LT_HEAL_END);
}

static void trace_remove(uint8_t offset)
{
    uint8_t i;
    for (i = offset; (uint8_t)(i + 1U) < s_size; i++) {
        s_events[(s_head + i) % LORA_TRACE_CAPACITY] =
            s_events[(s_head + i + 1U) % LORA_TRACE_CAPACITY];
    }
    s_size--; s_lost++;
}

static void radio_remove(uint8_t offset)
{
    uint8_t i;
    for (i = offset; (uint8_t)(i + 1U) < s_radio_size; i++) {
        s_radio[(s_radio_head + i) % RF_TRACE_CAPACITY] =
            s_radio[(s_radio_head + i + 1U) % RF_TRACE_CAPACITY];
    }
    s_radio_size--; s_radio_lost++;
}

void LoraTrace_Radio(const LoraTraceContext *ctx, uint8_t reason,
                      uint32_t seq, const RadioDiagSnapshot *snapshot)
{
    RadioTraceEvent *e;
    uint8_t at, i, key, normal;
    if (ctx == NULL || snapshot == NULL || reason >= RF_REASON_COUNT) { return; }
    s_radio_number++;
    key = (uint8_t)(reason == RF_HEAL_BEFORE || reason == RF_HEAL_AFTER);
    if (key == 0U && seq != 0U) {
        if (s_radio_seen_seq != seq) {
            s_radio_seen_seq = seq; s_radio_seen = 0U; s_radio_first_timeout = 0U;
        }
        if ((s_radio_seen & (1U << reason)) != 0U) { s_radio_compact++; return; }
    }
    /* Keep the latest recovery pair even through repeated recoveries. */
    if (key != 0U) {
        for (i = 0U; i < s_radio_size; i++) {
            if (s_radio[(s_radio_head + i) % RF_TRACE_CAPACITY].reason == reason) { break; }
        }
        if (i != s_radio_size) { radio_remove(i); }
    }
    normal = 0U;
    for (i = 0U; i < s_radio_size; i++) {
        uint8_t old = s_radio[(s_radio_head + i) % RF_TRACE_CAPACITY].reason;
        if (old != RF_HEAL_BEFORE && old != RF_HEAL_AFTER) { normal++; }
    }
    if (s_radio_size >= RF_TRACE_CAPACITY ||
        (key == 0U && normal >= RF_TRACE_CAPACITY - RF_KEY_RESERVE)) {
        for (i = 0U; i < s_radio_size; i++) {
            const RadioTraceEvent *old = &s_radio[(s_radio_head + i) % RF_TRACE_CAPACITY];
            if (old->reason != RF_HEAL_BEFORE && old->reason != RF_HEAL_AFTER &&
                !(old->reason == RF_CONFIRM_TIMEOUT && old->seq == s_radio_seen_seq &&
                  old->number == s_radio_first_timeout)) { break; }
        }
        if (i == s_radio_size) { s_radio_lost++; return; }
        radio_remove(i);
    }
    at = (uint8_t)((s_radio_head + s_radio_size) % RF_TRACE_CAPACITY);
    e = &s_radio[at]; e->snapshot = *snapshot; e->seq = seq;
    e->number = s_radio_number; e->reason = reason;
    e->part = ctx->part; e->host = ctx->host; e->tx = ctx->tx; e->rx = ctx->rx;
    s_radio_size++;
    if (key == 0U && seq != 0U) { s_radio_seen |= (uint8_t)(1U << reason); }
    if (reason == RF_CONFIRM_TIMEOUT && seq != 0U && s_radio_first_timeout == 0U) {
        s_radio_first_timeout = e->number;
    }
    /* Pending seq survives recovery: its first post-recovery attempt must be
     * observable. The original first timeout remains pinned until a new seq. */
    if (reason == RF_HEAL_AFTER) { s_radio_seen = 0U; }
}

/* FNV-1a is a correlation hint, not a sample ID or collision-proof checksum. */
uint32_t LoraTrace_DataKey(const uint8_t *data, uint16_t len)
{
    uint32_t hash = 2166136261UL;
    uint16_t i;
    if (data == NULL) { return 0U; }
    for (i = 0U; i < len; i++) { hash = (hash ^ data[i]) * 16777619UL; }
    return hash;
}

void LoraTrace_Record(const LoraTraceContext *ctx, uint8_t event,
                      uint32_t seq, const uint8_t *frame, uint16_t len,
                      uint16_t detail)
{
    TraceEvent captured;
    TraceEvent *e = &captured;
    uint8_t at, i, key, repeat;
    if ((ctx == NULL) || (event >= LT_EVENT_COUNT)) { return; }
    s_count[event]++;
    memset(e, 0, sizeof(*e));
    e->rt = ctx->rt; e->seq = seq; e->event = event; e->number = ++s_number;
    e->part = ctx->part; e->host = ctx->host;
    e->tx = ctx->tx; e->rx = ctx->rx; e->forward = ctx->forward;
    e->attempt = ctx->attempt; e->detail = detail;
    e->len = (len <= 255U) ? (uint8_t)len : 255U;
    if ((frame != NULL) && ((event == LT_READY) || (event == LT_REPLACE)) && (len == 80U)) {
        e->src = ctx->host; e->origin = ctx->host; e->cmd = 0x0AU;
        e->src_part = ctx->part; e->origin_part = ctx->part;
        e->key = LoraTrace_DataKey(frame, 80U);
        e->value_type = frame[0];
        e->value = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
    } else if (frame != NULL) {
        if (len >= 4U) {
            e->dst_part = frame[0]; e->dst = frame[1];
            e->src_part = frame[2]; e->src = frame[3];
        }
        if (len >= 10U) { e->cmd = frame[9]; }
        if (len >= 14U) { e->origin_part = frame[12]; e->origin = frame[13]; }
        /* 0x0A carries 80 channel bytes at [14..93]. These bytes survive
         * forwarding; routing/RSSI/voltage/header/checksum are excluded.
         * Identical values across different samples deliberately have one key. */
        if ((len == 95U) && (frame[9] == 0x0AU)) {
            e->key = LoraTrace_DataKey(&frame[14], 80U);
            e->value_type = frame[14];
            e->value = (uint16_t)(((uint16_t)frame[15] << 8) | frame[16]);
        }
    }
    if ((event == LT_SELF_TX) || (event == LT_FORWARD_TX)) {
        s_last_tx = *e;
    } else if ((event == LT_TX_EXIT) || (event == LT_SELF_TIMEOUT) ||
               (event == LT_FORWARD_TIMEOUT) || (event == LT_FORWARD_FAIL)) {
        /* Tx_buf is shared with network-test replies. Keep the last actual
         * business attempt's identity/signature even if that buffer changes. */
        e->src = s_last_tx.src; e->dst = s_last_tx.dst;
        e->origin = s_last_tx.origin; e->cmd = s_last_tx.cmd;
        e->src_part = s_last_tx.src_part; e->dst_part = s_last_tx.dst_part;
        e->origin_part = s_last_tx.origin_part; e->len = s_last_tx.len;
        e->key = s_last_tx.key; e->value = s_last_tx.value;
        e->value_type = s_last_tx.value_type; e->attempt = s_last_tx.attempt;
    }
    if (event == LT_READY || event == LT_REPLACE || event == LT_HEAL_END) {
        s_repeat_seen = 0U;
    }
    repeat = (uint8_t)(event == LT_SELF_TX || event == LT_SELF_TIMEOUT ||
        event == LT_CAD_RX_END || (event == LT_TX_EXIT && s_last_tx.event == LT_SELF_TX));
    if (repeat != 0U && (s_repeat_seen & (1UL << event)) != 0U) {
        s_compact++; return;
    }
    key = trace_is_key(event);
    if (key == 0U && s_size >= LORA_TRACE_CAPACITY - TRACE_KEY_RESERVE) {
        s_lost++; return;
    }
    if (s_size >= LORA_TRACE_CAPACITY) {
        for (i = 0U; i < s_size; i++) {
            if (trace_is_key(s_events[(s_head + i) % LORA_TRACE_CAPACITY].event) == 0U) { break; }
        }
        if (i == s_size && event != LT_SELF_TIMEOUT && event != LT_FORWARD_FAIL) {
            for (i = 0U; i < s_size; i++) {
                if (s_events[(s_head + i) % LORA_TRACE_CAPACITY].event == event) { break; }
            }
        }
        if (i == s_size) { s_lost++; return; }
        trace_remove(i);
    }
    at = (uint8_t)((s_head + s_size) % LORA_TRACE_CAPACITY);
    s_events[at] = *e;
    s_size++;
    if (repeat != 0U) { s_repeat_seen |= (1UL << event); }
}

void LoraTrace_Service(const LoraTraceContext *ctx, uint8_t allow_output)
{
    TraceEvent e;
    if (ctx == NULL) { return; }
    if ((s_started == 0U) || ((uint32_t)(ctx->rt - s_last_status) >= 600U)) {
        s_started = 1U; s_last_status = ctx->rt; s_status_due = 1U; s_counts_due = 1U;
    }
    if (allow_output == 0U) { return; }
    if (s_status_due != 0U) {
        s_status_due = 0U;
        printf("[LS] fw=" LORA_TRACE_BUILD " p=%u h=%u rt=%lu unit=100ms q=%u lost=%lu n=%lu rx=%lu bad=%lu fq=%lu busy=%lu to=%lu cad=%lu hit=%lu st=%u/%u pend=%u fwd=%u rq=%u rlost=%lu compact=%lu rcompact=%lu\r\n",
            (unsigned)ctx->part, (unsigned)ctx->host, (unsigned long)ctx->rt,
            (unsigned)s_size, (unsigned long)s_lost, (unsigned long)s_number, (unsigned long)s_count[LT_RX],
            (unsigned long)(s_count[LT_BAD_LEN] + s_count[LT_BAD_SUM]),
            (unsigned long)s_count[LT_FORWARD_QUEUE], (unsigned long)s_count[LT_FORWARD_BUSY],
            (unsigned long)(s_count[LT_SELF_TIMEOUT] + s_count[LT_FORWARD_TIMEOUT]),
            (unsigned long)ctx->cad, (unsigned long)ctx->hit,
            (unsigned)ctx->tx, (unsigned)ctx->rx,
            (unsigned)ctx->pending, (unsigned)ctx->forward,
            (unsigned)s_radio_size, (unsigned long)s_radio_lost,
            (unsigned long)s_compact, (unsigned long)s_radio_compact);
        return;
    }
    if (s_counts_due != 0U) {
        s_counts_due = 0U;
        printf("[LC] rt=%lu p=%u h=%u ready=%lu replace=%lu selftx=%lu fwdtx=%lu txexit=%lu rx=%lu stmo=%lu ftmo=%lu ffail=%lu sok=%lu fok=%lu hbegin=%lu hend=%lu watch=%lu\r\n",
            (unsigned long)ctx->rt, (unsigned)ctx->part, (unsigned)ctx->host,
            (unsigned long)s_count[LT_READY], (unsigned long)s_count[LT_REPLACE],
            (unsigned long)s_count[LT_SELF_TX], (unsigned long)s_count[LT_FORWARD_TX],
            (unsigned long)s_count[LT_TX_EXIT], (unsigned long)s_count[LT_RX],
            (unsigned long)s_count[LT_SELF_TIMEOUT], (unsigned long)s_count[LT_FORWARD_TIMEOUT],
            (unsigned long)s_count[LT_FORWARD_FAIL],
            (unsigned long)(s_count[LT_SELF_OK_D] + s_count[LT_SELF_OK_A] + s_count[LT_SELF_OK_OD]),
            (unsigned long)(s_count[LT_FORWARD_OK_D] + s_count[LT_FORWARD_OK_A] + s_count[LT_FORWARD_OK_OD]),
            (unsigned long)s_count[LT_HEAL_BEGIN], (unsigned long)s_count[LT_HEAL_END],
            (unsigned long)s_count[LT_WATCHDOG]);
        return;
    }
    if (s_radio_size != 0U) {
        const RadioTraceEvent *r = &s_radio[s_radio_head];
        const RadioDiagSnapshot *s = &r->snapshot;
        printf("[RF] n=%lu rt=%lu p=%u h=%u why=%s seq=%lu st=%u/%u mode=%u status=%02X irq=%04X err=%04X rd=%u pin=%02X imr=%02X pr=%02X bt=%u sb=%u tt=%u sp=%lu d1=%lu d2=%lu d3=%lu cd=%lu ca=%lu ri=%lu\r\n",
            (unsigned long)r->number, (unsigned long)s->rt, (unsigned)r->part,
            (unsigned)r->host, s_radio_names[r->reason], (unsigned long)r->seq,
            (unsigned)r->tx, (unsigned)r->rx, (unsigned)s->sw_mode,
            (unsigned)s->status, (unsigned)s->irq, (unsigned)s->errors,
            (unsigned)s->read_flags, (unsigned)s->pins,
            (unsigned)s->exti_mask, (unsigned)s->exti_pending,
            (unsigned)s->busy_to, (unsigned)s->standby_to, (unsigned)s->tx_to,
            (unsigned long)s->spi_fail, (unsigned long)s->dio1,
            (unsigned long)s->dio2, (unsigned long)s->dio3,
            (unsigned long)s->cad_done, (unsigned long)s->cad_activity,
            (unsigned long)s->rx_irq);
        s_radio_head = (uint8_t)((s_radio_head + 1U) % RF_TRACE_CAPACITY);
        s_radio_size--;
        return;
    }
    if (s_size == 0U) { return; }
    e = s_events[s_head];
    s_head = (uint8_t)((s_head + 1U) % LORA_TRACE_CAPACITY); s_size--;
    printf("[LK] n=%lu rt=%lu p=%u h=%u e=%s seq=%lu src=%u/%u dst=%u/%u orig=%u/%u cmd=%02X len=%u key=%08lX v0=%u:%u a=%u st=%u/%u f=%u x=%u\r\n",
        (unsigned long)e.number, (unsigned long)e.rt, (unsigned)e.part, (unsigned)e.host, s_names[e.event],
        (unsigned long)e.seq, (unsigned)e.src_part, (unsigned)e.src,
        (unsigned)e.dst_part, (unsigned)e.dst, (unsigned)e.origin_part, (unsigned)e.origin,
        (unsigned)e.cmd, (unsigned)e.len, (unsigned long)e.key,
        (unsigned)e.value_type, (unsigned)e.value, (unsigned)e.attempt,
        (unsigned)e.tx, (unsigned)e.rx, (unsigned)e.forward, (unsigned)e.detail);
}
