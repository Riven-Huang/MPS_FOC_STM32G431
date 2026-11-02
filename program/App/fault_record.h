#ifndef FAULT_RECORD_H
#define FAULT_RECORD_H
#include "motor.h"

/* Called only by the control ISR, before fault handling resets feedback. */
static void motor_capture_fault(volatile motor_fb_t *fb, const volatile motor_cmd_t *cmd,
                                uint16_t bits, uint32_t ms, float raw_v, uint16_t adc)
{
    uint32_t seq;
    volatile motor_fault_record_t *record;
    if (bits == 0U) return;
    seq = fb->fault_event_seq + 1U;
    if (seq == 0U) seq = 1U;
    record = &fb->fault_history[(seq - 1U) % MOTOR_FAULT_HISTORY_SIZE];
    record->seq = seq;
    record->time_ms = ms;
    record->bits = bits;
    record->vbus_raw = adc;
    record->vbus_v = fb->vbus_v;
    record->vbus_unfiltered_v = raw_v;
    record->ia_a = fb->ia_a; record->ib_a = fb->ib_a; record->ic_a = fb->ic_a;
    record->oc_a = cmd->prot_oc_a; record->ov_v = cmd->prot_ov_v; record->uv_v = cmd->prot_uv_v;
    fb->fault_event_seq = seq;
}
#endif
