#ifndef RADIO_DIAG_H
#define RADIO_DIAG_H
#include <stdint.h>

typedef struct {
    /* Classified by software state at ISR entry. cad_done counts DIO3 in
     * state 5/6; it is not a hardware-latched total of every CAD completion. */
    uint32_t dio1, dio2, dio3, cad_done, cad_activity, rx_irq;
} RadioDiagIrqCounters;
extern volatile RadioDiagIrqCounters g_radio_diag_irq;
extern volatile uint32_t g_radio_spi_fail_count;

typedef struct {
    uint32_t rt, dio1, dio2, dio3, cad_done, cad_activity, rx_irq, spi_fail;
    uint16_t irq, errors, busy_to, standby_to, tx_to, exti_mask, exti_pending;
    uint8_t status, sw_mode, pins, read_flags;
} RadioDiagSnapshot;

/* Main-loop only. read_flags bits 0/1/2 mark status/IRQ/error reads whose
 * SPI/BUSY counters stayed unchanged. These are transport checks, not proof
 * of physical delivery. Never wakes a sleeping/unknown/duty-cycle radio.
 * pins: bit0 BUSY, bit1 DIO1(PC4), bit2 DIO2(PC5), bit3 DIO3(PB0),
 * bit4 SW1(PB1), bit5 SW2(PB2). EXTI values retain lines 0/4/5 only. */
void SX126xCaptureDiagnostic(RadioDiagSnapshot *snapshot);
#endif
