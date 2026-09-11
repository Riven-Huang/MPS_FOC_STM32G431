#ifndef MOTOR_H
#define MOTOR_H

/* ============================================================================
 * motor.h — 对外统一接口
 *
 * 使用模型只有一句话：【写 g_cmd，读 g_fb】
 *
 *   g_cmd  用户/调试器写入：模式、总开关、各模式目标值、调参量
 *   g_fb   程序输出：状态、故障、全部测量量（只读，不要在用户侧写）
 *
 * 不再有 current_loop_enable / speed_loop_enable / position_loop_enable
 * 之类的布尔组合，一个 g_cmd.mode 决定电机工作在哪种模式。
 * ==========================================================================*/

#include <stdint.h>

#include "foc_core.h"

/* ---------------------------- 运行/测试模式 ---------------------------- */
typedef enum
{
    MOTOR_MODE_DISABLED = 0,    /* 功率级关闭，PWM 保持 50%，安全态 */
    MOTOR_MODE_PWM_TEST,        /* 固定占空比发波测试（验证 TIM1 + MP6539B，建议断开电机） */
    MOTOR_MODE_VOLTAGE,         /* 开环电压模式：ud/uq 直给（电角度来源见 volt_open_angle_en） */
    MOTOR_MODE_CURRENT,         /* 电流环：id/iq 直给（转矩模式） */
    MOTOR_MODE_SPEED,           /* 速度环：spd_rpm（转子侧转速） */
    MOTOR_MODE_POSITION,        /* 位置环：pos_deg（输出轴角度） */
} motor_mode_t;

/* ---------------------------- 状态机 ---------------------------- */
typedef enum
{
    MOTOR_STATE_IDLE = 0,       /* 待机：功率级关闭，等待 run */
    MOTOR_STATE_ALIGN,          /* 编码器零位对齐中（d 轴电压锁定转子） */
    MOTOR_STATE_RUN,            /* 运行中，按 mode_active 分发控制 */
    MOTOR_STATE_FAULT,          /* 故障锁存：功率级关闭，等待 clear_fault */
} motor_state_id_t;

/* ---------------------------- 故障位（可叠加） ---------------------------- */
#define FAULT_BIT_DRIVER        0x0001U     /* MP6539B N_FAULT 低有效 */
#define FAULT_BIT_OVERCURRENT   0x0002U     /* 相电流超过 CFG_PROT_PHASE_OC_A */
#define FAULT_BIT_OVERVOLT      0x0004U     /* 母线过压 */
#define FAULT_BIT_UNDERVOLT     0x0008U     /* 母线欠压 */
#define FAULT_BIT_ENCODER       0x0010U     /* 运行中编码器读数失效 */

/* ---------------------------- 命令结构体（用户写） ---------------------------- */
typedef struct
{
    /* --- 主控 --- */
    motor_mode_t mode;          /* 工作模式，见 motor_mode_t */
    uint8_t  run;               /* 总开关：0=停止回 IDLE，1=按 mode 启动（最后置位！） */
    uint8_t  clear_fault;       /* 写 1 清除故障（故障源消失后才有效），同时自动将 run 清 0，需重新置 run 启动 */

    /* --- PWM_TEST 模式 --- */
    float duty_a;               /* 三相固定占空比 0.0~1.0 */
    float duty_b;
    float duty_c;

    /* --- VOLTAGE 模式 --- */
    float ud_v;                 /* d 轴电压 V */
    float uq_v;                 /* q 轴电压 V */
    uint8_t  volt_open_angle_en;/* 0=用编码器闭环角度（需先对齐）；1=开环积分角度（不依赖编码器） */
    float open_spd_elec_rad_s;  /* 开环角度的电角速度 rad/s，仅在 volt_open_angle_en=1 时生效 */

    /* --- CURRENT 模式 --- */
    float id_a;                 /* d 轴电流目标 A（正常用 0） */
    float iq_a;                 /* q 轴电流目标 A（≈ 转矩） */

    /* --- SPEED 模式 --- */
    float spd_rpm;              /* 转子侧目标转速 rpm */

    /* --- POSITION 模式 --- */
    float pos_deg;              /* 输出轴目标角度 deg（0~360） */
    uint8_t  pos_hold_en;       /* 位置保持/爬行开关：1=启用（默认）；0=旁路（诊断抖动来源用） */

    /* --- 调参量（上电默认值见 app_config.h） --- */
    float iq_lim_a;             /* 电流限幅 A（速度环输出限幅、电流给定限幅共用） */
    float cur_kp;               /* 电流环 PI */
    float cur_ki;
    float spd_kp;               /* 速度环 PI */
    float spd_ki;
    float spd_lpf_hz;           /* 测速低通截止频率 */
    float pos_kp;               /* 位置环 P */
    float pos_ki;               /* 位置环 I（建议保持 0） */
    float pos_kd;               /* 位置环 D（测速阻尼） */
    float pos_spd_lim_rad_s;    /* 位置环输出（输出轴角速度）限幅 */
} motor_cmd_t;

/* ---------------------------- 反馈结构体（用户只读） ---------------------------- */
typedef struct
{
    /* --- 状态 --- */
    motor_state_id_t state;     /* 状态机当前状态 */
    motor_mode_t mode_active;   /* 实际生效的模式（IDLE 时为 DISABLED） */
    uint16_t fault_bits;        /* 故障位掩码，见 FAULT_BIT_* */
    uint8_t  align_done;        /* 编码器零位对齐完成 */
    uint8_t  offset_ready;      /* 电流零偏校准完成 */
    uint8_t  enc_valid;         /* 编码器数据有效 */
    uint8_t  drv_fault;         /* N_FAULT 当前电平（1=故障） */

    /* --- 原始采样 --- */
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint16_t vbus_raw;
    uint16_t ntc_raw;
    uint16_t enc_raw;           /* MA600A 原始角度码 0~65535 */

    /* --- 电流 --- */
    float ia_a;                 /* 三相相电流（A 相），流入电机为正 */
    float ib_a;
    float ic_a;
    float i_sum_a;              /* ia+ib+ic，正常应接近 0 */
    float id_a;                 /* 旋转坐标系 d 轴电流 */
    float iq_a;                 /* 旋转坐标系 q 轴电流 */
    float id_ref_a;             /* 实际送入 PI 的电流给定（限幅后） */
    float iq_ref_a;

    /* --- 角度 / 速度 / 位置 --- */
    float theta_elec_rad;       /* 当前控制用电角度 */
    float enc_deg;              /* 编码器机械角 0~360 */
    float spd_rot_rad_s;        /* 转子角速度 rad/s（滤波+量化抑制后） */
    float spd_rot_rpm;          /* 转子转速 rpm */
    float spd_out_rpm;          /* 输出轴转速 rpm = 转子 / 减速比 */
    float spd_ref_rpm;          /* 速度斜坡后实际送入速度环的给定 */
    float pos_out_rad;          /* 输出轴连续位置 rad */
    float pos_out_deg;          /* 输出轴位置 deg（0~360 包绕） */
    float pos_ref_deg;          /* 位置环当前目标 deg */
    float align_ofs_rad;        /* 对齐得到的电角度零位偏置 */

    /* --- 电压 / 温度 --- */
    float vbus_v;               /* 母线电压（滤波后） */
    float ntc_c;                /* NTC 温度 ℃（需先确认 CFG_NTC_PULLUP_OHM） */
    float ud_v;                 /* 实际输出 d 轴电压 */
    float uq_v;                 /* 实际输出 q 轴电压 */
    float volt_lim_v;           /* 当前电压矢量限幅 = 0.577 × vbus */
    float duty_a;               /* 实际下发三相占空比 */
    float duty_b;
    float duty_c;

    /* --- 实时性与编码器统计 --- */
    float loop_us;              /* 快环本次耗时 us（预算 100us） */
    float loop_us_max;          /* 快环历史最大耗时 us */
    uint32_t overrun_cnt;       /* 快环超时次数（应不增长） */
    uint32_t enc_samples;       /* 编码器累计有效样本 */
    uint32_t enc_reject;        /* 编码器跳变剔除次数 */
    uint32_t enc_err;           /* 编码器 SPI 错误次数 */
} motor_fb_t;

extern volatile motor_cmd_t g_cmd;
extern volatile motor_fb_t  g_fb;
extern foc_core_t g_foc;

#endif /* MOTOR_H */
