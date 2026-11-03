/* Execute the actual firmware parser and command handler with a fake UART. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor.h"
#include "app_config.h"
#include "fault_record.h"

volatile motor_cmd_t g_cmd;
volatile motor_fb_t g_fb;
foc_core_t g_foc;
static uint8_t input[4096];
static unsigned input_head, input_tail;
static uint8_t output[4096];
static unsigned output_len;

uint8_t cli_uart_read_byte(uint8_t *v)
{
    if (input_tail == input_head) return 0;
    *v = input[input_tail++];
    return 1;
}
void cli_uart_poll_rx(void) {}
uint8_t cli_uart_is_tx_busy(void) { return 0; }
uint8_t cli_uart_send_bytes(const uint8_t *p, uint16_t n)
{
    assert(output_len + n <= sizeof(output));
    memcpy(output + output_len, p, n);
    output_len += n;
    return 1;
}

#include "../../program/App/host_link.c"

static void frame(uint8_t op, uint8_t seq, const uint8_t *p, unsigned n, int corrupt)
{
    uint8_t *b = input;
    assert(!s_pending);
    input_head = n + 8;
    input_tail = 0;
    b[0] = 0xA5; b[1] = 0x5A; b[2] = op; b[3] = seq;
    write16(b + 4, n);
    if (n) memcpy(b + 6, p, n);
    write16(b + 6 + n, crc16(b + 2, n + 4) ^ corrupt);
}

static void command(uint8_t op, uint8_t seq, const uint8_t *p, unsigned n, uint32_t t)
{
    output_len = 0;
    frame(op, seq, p, n, 0);
    host_link_task(t);
    host_link_control_tick(t);
    host_link_task(t);
    if (output_len < 11 || output[2] != MSG_ACK) fprintf(stderr, "op=%u seq=%u output_len=%u kind=%u pending=%u reply=%u active=%u\n", op, seq, output_len, output[2], s_pending, s_reply, s_active);
    assert(output_len >= 11 && output[2] == MSG_ACK);
    assert(output[3] == seq && output[6] == op);
}

static void reset(void)
{
    memset((void *)&g_cmd, 0, sizeof(g_cmd));
    memset((void *)&g_fb, 0, sizeof(g_fb));
    host_link_init();
    input_head = input_tail = output_len = 0;
    g_fb.state = MOTOR_STATE_IDLE;
    g_fb.offset_ready = 1;
    g_fb.enc_valid = 1;
    g_fb.vbus_v = 48;
}

int main(int argc, char **argv)
{
    uint8_t config[92] = {4, 0, 1, 0};
    float v[22] = {0, 0, 240, 30, 0, .5, 200, 1, .5, 250, 10, 2, 150, 0, .3, .3, .4, .6, 16, 65, 12, 1500};
    float old_kp;
    uint8_t rate[2] = {2, 0};
    unsigned i;
    memcpy(config + 4, v, sizeof(v));
    reset();
    command(OP_RUN, 1, NULL, 0, 1);
    assert(output[7] == RES_NO_SESSION && !g_cmd.run);
    command(OP_HELLO, 2, NULL, 0, 2);
    assert(output[7] == RES_OK);
    command(OP_CONFIG, 3, config, sizeof(config), 3);
    assert(output[7] == RES_OK && g_cmd.mode == MOTOR_MODE_SPEED);
    assert(fabs(g_cmd.cur_kp - 0.6440265) < 1e-5);
    assert(fabs(g_cmd.spd_ki - 0.1409943) < 1e-5);
    if (argc > 1 && strcmp(argv[1], "--info") == 0) {
        output_len = 0;
        send_info();
        for (i = 0; i < output_len; i++) printf("%02x", output[i]);
        puts("");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "--event") == 0) {
        g_fb.vbus_v = 66.25f; g_fb.ia_a = 10.5f; g_fb.ib_a = -6; g_fb.ic_a = -4.5f;
        motor_capture_fault(&g_fb, &g_cmd, 6, 1234, 68.5f, 3400);
        g_fb.vbus_v = 48; g_fb.ia_a = 0; g_fb.fault_bits = 0;
        assert(g_fb.fault_history[0].vbus_v == 66.25f);
        assert(g_fb.fault_history[0].ia_a == 10.5f);
        output_len = 0; send_fault_event(1500);
        for (i = 0; i < output_len; i++) printf("%02x", output[i]);
        puts("");
        for (i = 0; i < 10; i++) motor_capture_fault(&g_fb, &g_cmd, 8, 2000+i, 11, 546);
        assert(g_fb.fault_event_seq == 11);
        assert(g_fb.fault_history[2].seq == 11);
        return 0;
    }
    /* Check actual firmware validation at both current boundaries. */
    {
        uint8_t field;
        unsigned indices[] = {0, 1, 7, 18};
        for (i = 0; i < 4; ++i) {
            unsigned k = indices[i];
            float saved = v[k];
            v[k] = 16;
            memcpy(config + 4, v, sizeof(v));
            assert(validate_config(config, &field) == RES_OK);
            v[k] = 16.01f;
            memcpy(config + 4, v, sizeof(v));
            assert(validate_config(config, &field) == RES_RANGE);
            v[k] = saved;
        }
        v[0] = -16; v[1] = -16;
        memcpy(config + 4, v, sizeof(v));
        assert(validate_config(config, &field) == RES_OK);
        v[1] = -16.01f;
        memcpy(config + 4, v, sizeof(v));
        assert(validate_config(config, &field) == RES_RANGE);
        v[0] = 0; v[1] = 0;
        memcpy(config + 4, v, sizeof(v));
        assert(fabs(CFG_SHUNT_OHM - 0.004) < 1e-9);
        assert(fabs(CFG_ADC_REF_V / CFG_ADC_FULL_SCALE / (CFG_CUR_GAIN * CFG_SHUNT_OHM) - 0.010073260073) < 3e-9);
    }
    /* Filter changes must be finite, within range, and IDLE-only. */
    v[21] = 1000;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 70, config, sizeof(config), 3);
    assert(output[7] == RES_OK && g_cmd.cur_lpf_hz == 1000);
    g_fb.state = MOTOR_STATE_RUN;
    v[21] = 0;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 71, config, sizeof(config), 3);
    assert(output[7] == RES_STATE && g_cmd.cur_lpf_hz == 1000);
    g_fb.state = MOTOR_STATE_IDLE;
    command(OP_CONFIG, 72, config, sizeof(config), 3);
    assert(output[7] == RES_OK && g_cmd.cur_lpf_hz == 0);
    v[21] = 5000;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 73, config, sizeof(config), 3);
    assert(output[7] == RES_OK && g_cmd.cur_lpf_hz == 5000);
    v[21] = 5001;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 74, config, sizeof(config), 3);
    assert(output[7] == RES_RANGE && g_cmd.cur_lpf_hz == 5000);
    v[21] = NAN;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 75, config, sizeof(config), 3);
    assert(output[7] == RES_RANGE && g_cmd.cur_lpf_hz == 5000);
    command(OP_CONFIG, 76, config, 88, 3);
    assert(output[7] == RES_LENGTH);
    v[21] = 1500;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 77, config, sizeof(config), 3);
    assert(output[7] == RES_OK && g_cmd.cur_lpf_hz == 1500);
    /* Protection changes: atomic, bounded, ordered, and IDLE-only. */
    assert(g_cmd.prot_oc_a == 16 && g_cmd.prot_ov_v == 65 && g_cmd.prot_uv_v == 12);
    v[18] = 6; v[19] = 58; v[20] = 20;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 60, config, sizeof(config), 3);
    assert(output[7] == RES_OK && g_cmd.prot_oc_a == 6 && g_cmd.prot_ov_v == 58);
    g_fb.state = MOTOR_STATE_RUN;
    v[18] = 7;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 61, config, sizeof(config), 3);
    assert(output[7] == RES_STATE && g_cmd.prot_oc_a == 6);
    g_fb.state = MOTOR_STATE_IDLE;
    v[18] = 16.1f;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 62, config, sizeof(config), 3);
    assert(output[7] == RES_RANGE && g_cmd.prot_oc_a == 6);
    v[18] = 6; v[20] = 58;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 63, config, sizeof(config), 3);
    assert(output[7] == RES_RANGE && g_cmd.prot_uv_v == 20);
    v[20] = 20; v[7] = 7;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 64, config, sizeof(config), 3);
    assert(output[7] == RES_RANGE && g_cmd.iq_lim_a == 1);
    v[7] = 1; v[18] = 10; v[19] = 65; v[20] = 12;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 65, config, sizeof(config), 3);
    assert(output[7] == RES_OK);
    v[19] = 80;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 66, config, sizeof(config), 3);
    assert(output[7] == RES_OK && g_cmd.prot_ov_v == 80);
    v[19] = 80.1f;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 67, config, sizeof(config), 3);
    assert(output[7] == RES_RANGE && g_cmd.prot_ov_v == 80);
    v[19] = 65;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 68, config, sizeof(config), 3);
    assert(output[7] == RES_OK);
    config[0] = 6;
    command(OP_CONFIG, 69, config, sizeof(config), 3);
    assert(output[7] == RES_RANGE && g_cmd.mode == MOTOR_MODE_SPEED);
    config[0] = MOTOR_MODE_SPEED;
    /* Malformed inputs must not partially alter the controller. */
    old_kp = g_cmd.cur_kp;
    v[9] = NAN;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 4, config, sizeof(config), 4);
    assert(output[7] == RES_RANGE && g_cmd.cur_kp == old_kp);
    command(OP_CONFIG, 5, config, 10, 5);
    assert(output[7] == RES_LENGTH);
    command(OP_RATE, 6, rate, 2, 6);
    assert(output[7] == RES_RANGE && s_period_ms == 5);
    /* CRC corruption cannot start a motor. */
    frame(OP_RUN, 7, NULL, 0, 1);
    host_link_task(7);
    host_link_control_tick(7);
    assert(!g_cmd.run);
    g_fb.vbus_v = 0;
    command(OP_RUN, 8, NULL, 0, 8);
    assert(output[7] == RES_NOT_READY && !g_cmd.run);
    g_fb.vbus_v = 48;
    command(OP_RUN, 9, NULL, 0, 9);
    assert(output[7] == RES_OK && g_cmd.run);
    /* A lost ACK retry must not re-start a stopped/faulted motor. */
    g_cmd.run = 0;
    g_fb.state = MOTOR_STATE_FAULT;
    command(OP_RUN, 9, NULL, 0, 10);
    assert(output[7] == RES_OK && !g_cmd.run);
    g_fb.state = MOTOR_STATE_RUN;
    v[9] = 125;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 10, config, sizeof(config), 11);
    assert(output[7] == RES_STATE && g_cmd.cur_kp == old_kp);
    v[9] = 250;
    v[2] = 480;
    memcpy(config + 4, v, sizeof(v));
    command(OP_CONFIG, 11, config, sizeof(config), 12);
    assert(output[7] == RES_OK && g_cmd.spd_rpm == 480);
    g_cmd.run = 1;
    host_link_control_tick(2012);
    assert(!g_cmd.run && !s_active && s_timed_out);
    command(OP_HELLO, 12, NULL, 0, 2013);
    assert(!g_cmd.run);
    command(OP_STOP, 13, NULL, 0, 2014);
    assert(!g_cmd.run);
    /* Legacy/local runs are not claimed merely by observing with HELLO. */
    g_cmd.run = 1;
    host_link_control_tick(4014);
    assert(g_cmd.run && !s_active);
    command(OP_STOP, 14, NULL, 0, 4015);
    assert(!g_cmd.run);
    puts("Firmware protocol checks passed");
    return 0;
}
