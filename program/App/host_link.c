#include "host_link.h"

#include <math.h>
#include <string.h>
#include "app_config.h"
#include "cli_uart.h"
#include "motor.h"
#include "units.h"

/* FOC serial v2: A5 5A, type, seq, payload length LE16, payload, CRC16 LE.
 * CRC-CCITT-FALSE covers type through payload. No packed C structs on wire.
 * 5-channel legacy JustFloat stays active until HELLO. See host_app/PROTOCOL.md. */
#define HOST_MAX_PAYLOAD 160U
#define HOST_CONFIG_SIZE 92U
#define HOST_TIMEOUT_MS 2000U
#define HOST_PARTIAL_MS 100U
#define OP_HELLO 1U
#define OP_CONFIG 2U
#define OP_RUN 3U
#define OP_STOP 4U
#define OP_CLEAR 5U
#define OP_PING 6U
#define OP_READ 7U
#define OP_RATE 8U
#define MSG_ACK 0x80U
#define MSG_INFO 0x81U
#define MSG_STATUS 0x82U
#define MSG_SAMPLE 0x83U
#define MSG_FAULT_EVENT 0x84U
#define RES_OK 0U
#define RES_LENGTH 1U
#define RES_RANGE 2U
#define RES_BUSY 3U
#define RES_STATE 4U
#define RES_NOT_READY 5U
#define RES_NO_SESSION 6U

static uint8_t s_rx[HOST_MAX_PAYLOAD + 8U];
static uint16_t s_rx_len;
static uint32_t s_rx_time;
static volatile uint8_t s_active;
static volatile uint8_t s_owned_run;
static volatile uint8_t s_timed_out;
static volatile uint32_t s_last_rx;
static volatile uint8_t s_pending;
static uint8_t s_op;
static uint8_t s_seq;
static uint8_t s_payload[HOST_MAX_PAYLOAD];
static uint16_t s_payload_len;
static volatile uint8_t s_reply;
static uint8_t s_result;
static uint8_t s_field;
static uint8_t s_cache_op;
static uint8_t s_cache_seq;
static uint16_t s_cache_len;
static uint8_t s_cache_payload[HOST_MAX_PAYLOAD];
static uint8_t s_cache_result;
static uint8_t s_cache_field;
static uint8_t s_cache_valid;
static uint8_t s_info_pending;
static uint8_t s_info_seq;
static uint16_t s_period_ms = 5U;
static uint32_t s_sample_time;
static uint32_t s_status_time;
static uint32_t s_event_time;
static uint8_t s_event_slot;

static uint16_t read16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void write16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void write32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void put_float(uint8_t *p, float value)
{
    memcpy(p, &value, sizeof(value));
}

static uint16_t crc16(const uint8_t *p, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t send_frame(uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[HOST_MAX_PAYLOAD + 8U];
    if (len > HOST_MAX_PAYLOAD) {
        return 0U;
    }
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = type;
    frame[3] = seq;
    write16(frame + 4, len);
    memcpy(frame + 6, payload, len);
    write16(frame + 6 + len, crc16(frame + 2, len + 4U));
    return cli_uart_send_bytes(frame, len + 8U);
}

static uint8_t needs_encoder(motor_mode_t mode, uint8_t open_angle)
{
    return (mode >= MOTOR_MODE_CURRENT) || ((mode == MOTOR_MODE_VOLTAGE) && !open_angle);
}

static uint8_t ready_to_run(void)
{
    return (g_fb.state == MOTOR_STATE_IDLE) && (g_fb.fault_bits == 0U) &&
        (g_fb.drv_fault == 0U) && (g_fb.offset_ready != 0U) &&
        isfinite(g_fb.vbus_v) && (g_fb.vbus_v >= g_cmd.prot_uv_v) &&
        (g_fb.vbus_v <= g_cmd.prot_ov_v) &&
        (!needs_encoder(g_cmd.mode, g_cmd.volt_open_angle_en) || g_fb.enc_valid);
}

/* Validation happens in foreground. Application is atomic at a control boundary. */
static uint8_t validate_config(const uint8_t *p, uint8_t *field)
{
    static const float lo[22] = {-CFG_CURRENT_MAX_A, -CFG_CURRENT_MAX_A, -6000, 0, -40, -40, -10000,
        0.05f, 0.01f, 10, 1, 0.1f, 1, 0, 0, 0, 0, 0, 0.5f, 13, 12, 0};
    static const float hi[22] = {CFG_CURRENT_MAX_A, CFG_CURRENT_MAX_A, 6000, 360, 40, 40, 10000,
        CFG_CURRENT_MAX_A, 20, 500, 60, 20, 1000, 100, 10, 1, 1, 1, CFG_CURRENT_MAX_A, CFG_PROT_VBUS_OV_MAX_V, 60, 5000};
    float v;
    uint8_t i;
    if ((p[0] > MOTOR_MODE_POSITION) || (p[1] > 1U) || (p[2] > 1U) || (p[3] != 0U)) {
        *field = 0U;
        return RES_RANGE;
    }
    for (i = 0U; i < 22U; i++) {
        memcpy(&v, p + 4U + 4U*i, 4U);
        if (!isfinite(v) || (v < lo[i]) || (v > hi[i])) {
            *field = i + 1U;
            return RES_RANGE;
        }
    }
    {
        float values[22];
        memcpy(values, p + 4, sizeof(values));
        if ((values[20] >= values[19]) || (values[7] > values[18])) {
            *field = (values[20] >= values[19]) ? 21U : 19U;
            return RES_RANGE;
        }
    }
    return RES_OK;
}

static uint8_t apply_config(void)
{
    float v[22];
    float wn;
    float cur_kp;
    float cur_ki;
    float spd_kp;
    float spd_ki;
    float pos_kp;
    memcpy(v, s_payload + 4, sizeof(v));
    cur_kp = UNITS_TWO_PI*v[9]*CFG_CUR_EQ_L_H;
    cur_ki = UNITS_TWO_PI*v[9]*CFG_CUR_EQ_R_OHM;
    wn = UNITS_TWO_PI*v[10];
    spd_kp = 2.0f*CFG_SPD_DAMPING*wn*CFG_MOTOR_INERTIA_KGM2/CFG_MOTOR_KT_NM_A/CFG_MOTOR_POLE_PAIRS;
    spd_ki = wn*wn*CFG_MOTOR_INERTIA_KGM2/CFG_MOTOR_KT_NM_A/CFG_MOTOR_POLE_PAIRS;
    pos_kp = UNITS_TWO_PI*v[11];
    /* Live target updates are allowed. Topology and gain changes require IDLE. */
    if ((g_fb.state != MOTOR_STATE_IDLE) &&
        ((g_cmd.mode != (motor_mode_t)s_payload[0]) ||
         (g_cmd.volt_open_angle_en != s_payload[1]) ||
         (fabsf(g_cmd.cur_kp - cur_kp) > 0.00001f) ||
         (fabsf(g_cmd.cur_ki - cur_ki) > 0.01f) ||
         (fabsf(g_cmd.spd_kp - spd_kp) > 0.000001f) ||
         (fabsf(g_cmd.spd_ki - spd_ki) > 0.00001f) ||
         (fabsf(g_cmd.pos_kp - pos_kp) > 0.0001f) ||
         (g_cmd.pos_ki != v[13]) || (g_cmd.pos_kd != v[14]) ||
         (g_cmd.prot_oc_a != v[18]) || (g_cmd.prot_ov_v != v[19]) ||
         (g_cmd.prot_uv_v != v[20]) || (g_cmd.cur_lpf_hz != v[21]) || (g_cmd.spd_lpf_hz != v[12]) || (g_cmd.pos_hold_en != s_payload[2]))) {
        return RES_STATE;
    }
    if (g_fb.state == MOTOR_STATE_ALIGN) {
        return RES_STATE;
    }
    g_cmd.mode = (motor_mode_t)s_payload[0];
    g_cmd.volt_open_angle_en = s_payload[1];
    g_cmd.pos_hold_en = s_payload[2];
    g_cmd.id_a = v[0];
    g_cmd.iq_a = v[1];
    g_cmd.spd_rpm = v[2];
    g_cmd.pos_deg = v[3];
    g_cmd.ud_v = v[4];
    g_cmd.uq_v = v[5];
    g_cmd.open_spd_elec_rad_s = v[6];
    g_cmd.iq_lim_a = v[7];
    g_cmd.pos_spd_lim_rad_s = v[8];
    g_cmd.cur_kp = cur_kp;
    g_cmd.cur_ki = cur_ki;
    g_cmd.spd_kp = spd_kp;
    g_cmd.spd_ki = spd_ki;
    g_cmd.pos_kp = pos_kp;
    g_cmd.cur_lpf_hz = v[21];
    g_cmd.spd_lpf_hz = v[12];
    g_cmd.pos_ki = v[13];
    g_cmd.pos_kd = v[14];
    g_cmd.duty_a = v[15];
    g_cmd.duty_b = v[16];
    g_cmd.duty_c = v[17];
    g_cmd.prot_oc_a = v[18];
    g_cmd.prot_ov_v = v[19];
    g_cmd.prot_uv_v = v[20];
    return RES_OK;
}

void host_link_control_tick(uint32_t now_ms)
{
    if (s_active && ((uint32_t)(now_ms - s_last_rx) >= HOST_TIMEOUT_MS)) {
        if (s_owned_run) {
            g_cmd.run = 0U;
        }
        s_owned_run = 0U;
        s_timed_out = 1U;
        s_active = 0U;
        s_pending = 0U;
        s_reply = 0U;
    }
    if (!s_pending) {
        return;
    }
    s_result = RES_OK;
    switch (s_op) {
    case OP_CONFIG:
        s_result = apply_config();
        break;
    case OP_RUN:
        if (!ready_to_run() || (g_cmd.mode == MOTOR_MODE_DISABLED)) {
            s_result = RES_NOT_READY;
        } else {
            g_cmd.run = 1U;
            s_owned_run = 1U;
        }
        break;
    case OP_STOP:
        g_cmd.run = 0U;
        s_owned_run = 0U;
        break;
    case OP_CLEAR:
        g_cmd.run = 0U;
        s_owned_run = 0U;
        if (g_fb.drv_fault || !isfinite(g_fb.vbus_v) ||
            (g_fb.vbus_v < g_cmd.prot_uv_v) || (g_fb.vbus_v > g_cmd.prot_ov_v) ||
            (fabsf(g_fb.ia_a) > g_cmd.prot_oc_a) ||
            (fabsf(g_fb.ib_a) > g_cmd.prot_oc_a) ||
            (fabsf(g_fb.ic_a) > g_cmd.prot_oc_a)) {
            s_result = RES_NOT_READY;
        } else {
            g_cmd.clear_fault = 1U;
        }
        break;
    default:
        s_result = RES_RANGE;
        break;
    }
    s_pending = 0U;
    s_reply = 1U;
}

static void receive_frame(uint32_t now_ms)
{
    uint8_t op = s_rx[2];
    uint8_t seq = s_rx[3];
    uint16_t len = read16(s_rx + 4);
    uint8_t result = RES_OK;
    uint16_t period;
    if (s_pending || s_reply) {
        /* Stop preempts pending commands. Other frames are retried by the host. */
        if ((op != OP_STOP) || (len != 0U)) {
            return;
        }
        s_pending = 0U;
    }
    s_op = op;
    s_seq = seq;
    s_payload_len = len;
    memcpy(s_payload, s_rx + 6, len);
    s_field = 0U;
    if ((op != OP_HELLO) && (op != OP_STOP) && !s_active) {
        result = RES_NO_SESSION;
    } else if ((op == OP_CONFIG) && (len == HOST_CONFIG_SIZE)) {
        result = validate_config(s_payload, &s_field);
    } else if ((op == OP_RATE) && (len == 2U)) {
        period = read16(s_payload);
        if ((period < 5U) || (period > 1000U)) {
            result = RES_RANGE;
        }
    } else if ((op < OP_HELLO) || (op > OP_RATE)) {
        result = RES_RANGE;
    } else if ((op == OP_CONFIG) || (op == OP_RATE) || (len != 0U)) {
        result = RES_LENGTH;
    }
    if (result == RES_OK) {
        s_last_rx = now_ms;
        if (s_cache_valid && (op != OP_HELLO) && (op != OP_READ) && (op != OP_STOP) &&
            (op == s_cache_op) && (seq == s_cache_seq) && (len == s_cache_len) &&
            (memcmp(s_payload, s_cache_payload, len) == 0)) {
            s_result = s_cache_result;
            s_field = s_cache_field;
            s_reply = 1U;
            return;
        }
        switch (op) {
        case OP_HELLO:
            s_cache_valid = 0U;
            s_active = 1U;
            s_timed_out = 0U;
            s_info_pending = 1U;
            s_info_seq = seq;
            break;
        case OP_READ:
            s_info_pending = 1U;
            s_info_seq = seq;
            break;
        case OP_RATE:
            s_period_ms = read16(s_payload);
            break;
        case OP_PING:
            break;
        default:
            s_pending = 1U; /* Publish last, ADC interrupt consumes the mailbox. */
            return;
        }
    }
    s_result = result;
    s_reply = 1U;
}

static void send_info(void)
{
    uint8_t p[156];
    motor_cmd_t cmd;
    uint32_t mask;
    float meta[8] = {CFG_CUR_EQ_R_OHM, CFG_CUR_EQ_L_H, CFG_MOTOR_KT_NM_A,
        CFG_MOTOR_INERTIA_KGM2, CFG_MOTOR_POLE_PAIRS, CFG_MOTOR_GEAR_RATIO,
        CFG_SPD_DAMPING, CFG_FAST_LOOP_HZ};
    float v[22];
    float gains[7];
    mask = __get_PRIMASK();
    __disable_irq();
    cmd = g_cmd;
    __set_PRIMASK(mask);
    p[0] = 4U;
    p[1] = 0U;
    write16(p + 2, s_period_ms);
    memcpy(p + 4, meta, sizeof(meta));
    p[36] = (uint8_t)cmd.mode;
    p[37] = cmd.volt_open_angle_en;
    p[38] = cmd.pos_hold_en;
    p[39] = 0U;
    v[0] = cmd.id_a; v[1] = cmd.iq_a; v[2] = cmd.spd_rpm; v[3] = cmd.pos_deg;
    v[4] = cmd.ud_v; v[5] = cmd.uq_v; v[6] = cmd.open_spd_elec_rad_s;
    v[7] = cmd.iq_lim_a; v[8] = cmd.pos_spd_lim_rad_s;
    v[9] = cmd.cur_kp/(UNITS_TWO_PI*CFG_CUR_EQ_L_H);
    v[10] = cmd.spd_kp*CFG_MOTOR_KT_NM_A*CFG_MOTOR_POLE_PAIRS /
        (2.0f*CFG_SPD_DAMPING*UNITS_TWO_PI*CFG_MOTOR_INERTIA_KGM2);
    v[11] = cmd.pos_kp/UNITS_TWO_PI;
    v[12] = cmd.spd_lpf_hz; v[13] = cmd.pos_ki; v[14] = cmd.pos_kd;
    v[15] = cmd.duty_a; v[16] = cmd.duty_b; v[17] = cmd.duty_c;
    v[18] = cmd.prot_oc_a; v[19] = cmd.prot_ov_v; v[20] = cmd.prot_uv_v;
    v[21] = cmd.cur_lpf_hz;
    memcpy(p + 40, v, sizeof(v));
    gains[0] = cmd.cur_kp; gains[1] = cmd.cur_ki;
    gains[2] = cmd.spd_kp; gains[3] = cmd.spd_ki;
    gains[4] = cmd.pos_kp; gains[5] = cmd.pos_ki; gains[6] = cmd.pos_kd;
    memcpy(p + 128, gains, sizeof(gains));
    if (send_frame(MSG_INFO, s_info_seq, p, sizeof(p))) {
        s_info_pending = 0U;
    }
}

static void send_status(uint32_t now_ms)
{
    uint8_t p[68];
    uint32_t mask = __get_PRIMASK();
    write32(p, now_ms);
    __disable_irq();
    p[4] = (uint8_t)g_fb.state; p[5] = (uint8_t)g_cmd.mode;
    p[6] = g_cmd.run; p[7] = g_fb.align_done; p[8] = g_fb.offset_ready;
    p[9] = g_fb.enc_valid; p[10] = g_fb.drv_fault; p[11] = s_timed_out;
    write16(p + 12, g_fb.fault_bits);
    write16(p + 14, s_period_ms);
    put_float(p + 16, g_fb.vbus_v); put_float(p + 20, g_fb.ntc_c);
    put_float(p + 24, g_fb.ud_v); put_float(p + 28, g_fb.uq_v);
    put_float(p + 32, g_fb.volt_lim_v); put_float(p + 36, g_fb.id_ref_a);
    put_float(p + 40, g_fb.iq_ref_a); put_float(p + 44, g_fb.spd_ref_rpm);
    put_float(p + 48, g_fb.pos_ref_deg); put_float(p + 52, g_fb.loop_us);
    write32(p + 56, g_fb.enc_err); write32(p + 60, g_fb.enc_reject);
    write32(p + 64, g_fb.overrun_cnt);
    __set_PRIMASK(mask);
    if (send_frame(MSG_STATUS, 0U, p, sizeof(p))) {
        s_status_time = now_ms;
    }
}

static void send_fault_event(uint32_t now_ms)
{
    motor_fault_record_t record;
    uint8_t p[44];
    uint8_t i;
    uint32_t mask;
    s_event_time = now_ms;
    for (i = 0U; i < MOTOR_FAULT_HISTORY_SIZE; i++) {
        mask = __get_PRIMASK();
        __disable_irq();
        record = g_fb.fault_history[s_event_slot];
        __set_PRIMASK(mask);
        s_event_slot = (s_event_slot + 1U) % MOTOR_FAULT_HISTORY_SIZE;
        if (record.seq != 0U) {
            write32(p, record.seq); write32(p + 4, record.time_ms);
            write16(p + 8, record.bits); write16(p + 10, record.vbus_raw);
            put_float(p + 12, record.vbus_v); put_float(p + 16, record.vbus_unfiltered_v);
            put_float(p + 20, record.ia_a); put_float(p + 24, record.ib_a); put_float(p + 28, record.ic_a);
            put_float(p + 32, record.oc_a); put_float(p + 36, record.ov_v); put_float(p + 40, record.uv_v);
            (void)send_frame(MSG_FAULT_EVENT, 0U, p, sizeof(p));
            break;
        }
    }
}

static void send_sample(uint32_t now_ms)
{
    uint8_t p[24];
    uint32_t mask = __get_PRIMASK();
    write32(p, now_ms);
    __disable_irq();
    put_float(p + 4, g_fb.id_a); put_float(p + 8, g_fb.iq_a);
    put_float(p + 12, g_fb.spd_out_rpm); put_float(p + 16, g_fb.pos_out_deg);
    put_float(p + 20, g_fb.enc_deg);
    __set_PRIMASK(mask);
    if (send_frame(MSG_SAMPLE, 0U, p, sizeof(p))) {
        s_sample_time = now_ms;
    }
}

void host_link_init(void)
{
    s_active = 0U;
    s_owned_run = 0U;
    s_pending = 0U;
    s_reply = 0U;
    s_rx_len = 0U;
    s_cache_valid = 0U;
    s_info_pending = 0U;
    s_period_ms = 5U;
}

uint8_t host_link_task(uint32_t now_ms)
{
    uint8_t b;
    uint8_t ack[3];
    uint16_t len;
    uint16_t budget = 256U;
    cli_uart_poll_rx();
    if ((uint32_t)(now_ms - s_rx_time) > HOST_PARTIAL_MS) {
        s_rx_len = 0U;
    }
    while (budget-- && cli_uart_read_byte(&b)) {
        s_rx_time = now_ms;
        if ((s_rx_len == 0U) && (b != 0xA5U)) {
            continue;
        }
        if ((s_rx_len == 1U) && (b != 0x5AU)) {
            s_rx_len = (b == 0xA5U) ? 1U : 0U;
            continue;
        }
        s_rx[s_rx_len++] = b;
        if (s_rx_len >= 6U) {
            len = read16(s_rx + 4);
            if (len > HOST_MAX_PAYLOAD) {
                s_rx_len = 0U;
            } else if (s_rx_len == len + 8U) {
                if (read16(s_rx + 6 + len) == crc16(s_rx + 2, len + 4U)) {
                    receive_frame(now_ms);
                }
                s_rx_len = 0U;
            }
        }
    }
    if (s_pending || cli_uart_is_tx_busy()) {
        return s_active;
    }
    if (s_reply) {
        ack[0] = s_op; ack[1] = s_result; ack[2] = s_field;
        if (send_frame(MSG_ACK, s_seq, ack, sizeof(ack))) {
            s_cache_op = s_op; s_cache_seq = s_seq; s_cache_len = s_payload_len;
            memcpy(s_cache_payload, s_payload, s_payload_len);
            s_cache_result = s_result; s_cache_field = s_field; s_cache_valid = 1U;
            s_reply = 0U;
        }
    } else if (s_active && s_info_pending) {
        send_info();
    } else if (s_active && ((uint32_t)(now_ms - s_status_time) >= 200U)) {
        send_status(now_ms);
    } else if (s_active && g_fb.fault_event_seq && ((uint32_t)(now_ms - s_event_time) >= 200U)) {
        send_fault_event(now_ms);
    } else if (s_active && ((uint32_t)(now_ms - s_sample_time) >= s_period_ms)) {
        send_sample(now_ms);
    }
    return s_active;
}
