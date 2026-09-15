#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================================================================
 * app_config.h — 全项目唯一的参数出处
 *
 * 集中管理三类参数：
 *   1. 电机与机构参数（换电机时优先改这里）
 *   2. 板级硬件参数（采样链路、分压比、NTC）
 *   3. 控制默认值与保护阈值（运行时想改的量在 g_cmd 里，这里只是初值）
 *
 * 命名约定：CFG_<域>_<含义>_<单位>
 * ==========================================================================*/

/* ---------------- 电机与机构 ---------------- */
#define CFG_MOTOR_POLE_PAIRS        14.0f       /* 极对数 */
#define CFG_MOTOR_GEAR_RATIO        8.0f        /* 减速比，无减速器填 1 */
#define CFG_ENC_ON_OUTPUT_SHAFT     0U          /* 1=编码器在输出轴，0=在转子侧 */
#define CFG_ENC_DIR_SIGN            (-1.0f)     /* 编码器方向：电机正转时读数减小填 -1 */

/* 转子侧电机电磁参数（速度环按带宽整定用）：
 *   Kt：由 4.7 N.m/A(输出轴) / 8 = 0.59 推算，与转速常数互验：
 *       1.98 rpm/V(输出轴) × 8 = 15.84 rpm/V → Ke ≈ 0.60 V·s/rad，SI 下 Kt≈Ke
 *   J ：转子+减速器输入端等效惯量，【估计值】，是速度环的实际增益旋钮：
 *       速度环振荡就减小它，响应发软就增大它（或运行时直接改 spd_kp/ki） */
#define CFG_MOTOR_KT_NM_A           0.60f
#define CFG_MOTOR_INERTIA_KGM2      0.00030f

/* ---------------- 板级采样链 ---------------- */
#define CFG_ADC_REF_V               3.3f
#define CFG_ADC_FULL_SCALE          4095.0f
#define CFG_SHUNT_OHM               0.008f      /* 相电流采样电阻（V1.3 板为 8 mΩ） */
#define CFG_CUR_GAIN                20.0f       /* 电流采样放大倍数 */
/* 由上面两项决定的电流满量程：±(1.65V / (0.008*20)) ≈ ±10.3A，iq_lim 不要超过它 */
#define CFG_VBUS_R_UP_OHM           240000.0f   /* 母线分压上电阻 */
#define CFG_VBUS_R_DOWN_OHM         10000.0f    /* 母线分压下电阻 */
/* NTC 10K B3950：假设上拉电阻到 3.3V、NTC 到地。
 * TODO: 请对照 V1.3 原理图确认上拉阻值与拓扑 */
#define CFG_NTC_PULLUP_OHM          10000.0f
#define CFG_NTC_R25_OHM             10000.0f
#define CFG_NTC_BETA                3950.0f

/* 相电流极性：流入电机为正；某相方向反了就把对应符号改成 -1 */
#define CFG_CUR_SIGN_IA             (1.0f)
#define CFG_CUR_SIGN_IB             (1.0f)
#define CFG_CUR_SIGN_IC             (1.0f)

/* ---------------- 控制节拍（全部在同一 10kHz 中断上下文内分频执行） ---------------- */
#define CFG_FAST_LOOP_HZ            10000.0f    /* 快环频率（ADC1 注入组，电流环） */
#define CFG_FAST_DT_S               (1.0f / CFG_FAST_LOOP_HZ)
#define CFG_SLOW_DIV                10U         /* 慢任务分频：VBUS/NTC/慢保护/派生单位 → 1kHz */
#define CFG_SLOW_LOOP_HZ            (CFG_FAST_LOOP_HZ / (float)CFG_SLOW_DIV)
#define CFG_SLOW_DT_S               (1.0f / CFG_SLOW_LOOP_HZ)
#define CFG_POS_LOOP_HZ             200.0f      /* 位置环频率（快环分频） */
#define CFG_POS_DT_S                (1.0f / CFG_POS_LOOP_HZ)
#define CFG_SPD_WINDOW_SAMPLES      5U          /* 测速窗口（快环样本数）→ 速度环约 2kHz */
#define CFG_SPD_DT_S                (CFG_FAST_DT_S * (float)CFG_SPD_WINDOW_SAMPLES)

/* ---------------- 上电默认控制参数（运行时可经 g_cmd 修改） ---------------- */
#define CFG_IQ_LIM_A_DEFAULT        5.0f        /* 闭环调试上限；给过流保护留出超调余量 */
#define CFG_SPD_RPM_DEFAULT         0.0f
#define CFG_VOLT_LIMIT_RATIO        0.57735026919f  /* SVPWM 线性区：1/sqrt(3) */
#define CFG_SPD_RAMP_RAD_S2         2500.0f     /* 速度给定斜坡；估计最大加速度 Kt·iq_lim/J≈10000 rad/s² 的约 25% */
#define CFG_SPD_LPF_HZ_DEFAULT      300.0f      /* 测速一阶低通截止频率（配 5 样本窗：测速链总延迟约 1.1ms） */
#define CFG_POS_KI_DEFAULT          0.0f
#define CFG_POS_KD_DEFAULT          0.3f        /* 速度阻尼：耗散突破静摩擦后的冲击，抑制位置回正抖动 */
#define CFG_POS_HOLD_EN_DEFAULT     1U          /* 位置保持/爬行开关默认值，运行时可经 g_cmd.pos_hold_en 改 */
#define CFG_POS_SPD_LIM_RAD_S       8.0f        /* 位置环输出轴速度上限 ≈76rpm（48V 空载上限约 95rpm）。
                                                 * 急减速能量回灌母线，若报过压(0x04) 则调小；运行时经 g_cmd 可改 */

/* 当前闭环调试目标带宽：电流环 500Hz / 速度环 30Hz / 位置环 5Hz，级联比约 100:6:1。
 * 该目标依赖已辨识的电机/机械参数、足够低的测速延迟和未饱和的电流执行器，
 * 不代表新板上电即可稳定运行。
 * 注意 1：电流环带宽受 PWM 更新率（20kHz / RCR(3+1) = 5kHz）+ 采样计算延迟
 *   （合计约 0.25ms）限制：800Hz 时相位裕度仅约 18°，闭环振铃（可闻噪声、
 *   阶跃超调 >60%，曾诱发过流保护），500Hz 约 45°。需要 800Hz 以上必须先
 *   在 CubeMX 把 TIM1 的 RCR 改小以提高 PWM 更新率，再回头提带宽。
 * 注意 2：速度反馈链（SPI 样本龄约 0.1ms + 5 点测速窗群延迟 0.25ms
 *   + 300Hz LPF 约 0.53ms + 2kHz 零阶保持约 0.25ms）等效延迟约 1.1ms，
 *   速度环带宽上限约 60Hz，30Hz 有充足裕度。
 * 电流环 PI（对象 R/L 串联模型，模最佳整定）：
 *   Kp = 2π·bw·L，Ki = 2π·bw·R（等效对象参数见 README 9.2） */
#define CFG_CUR_BW_HZ_DEFAULT       250.0f
#define CFG_CUR_EQ_R_OHM            0.7250f
#define CFG_CUR_EQ_L_H              0.0004100f

/* 速度环 PI（对象 Kt/J 积分模型，二阶整定：ωn=2π·bw，阻尼 ζ）：
 *   Kp = 2ζ·ωn·J/Kt，Ki = ωn²·J/Kt   （机械 rad/s 域）
 *   代码内 PI 在电角速度域运行（给定/反馈同乘极对数），增益需再除以极对数。
 *   当前：ωn=2π×30，Kp≈0.01346，Ki≈1.26895 */
#define CFG_SPD_BW_HZ_DEFAULT       10.0f
#define CFG_SPD_DAMPING             1.0f

/* 位置环 P（内环 30Hz 快于本环，对象近似纯积分 1/s）：
 *   Kp = 2π·bw（输出轴 rad 域），当前：2π×5 ≈ 31.42 */
#define CFG_POS_BW_HZ_DEFAULT       2.0f

/* ---------------- 位置环 hold/creep（抑制静摩擦导致的低频抖动） ---------------- */
#define CFG_POS_HOLD_ERR_RAD        0.021f      /* 进入保持：|误差| < ~1.2° */
#define CFG_POS_HOLD_REL_ERR_RAD    0.031f      /* 退出保持：|误差| > ~1.8° */
#define CFG_POS_HOLD_SPD_RAD_S      0.50f       /* 进入保持的速度条件 */
#define CFG_POS_HOLD_REL_CYCLES     12U         /* 退出保持的确认拍数 */
#define CFG_POS_CREEP_ERR_RAD       0.045f      /* 超过 ~2.6° 且指令过小则给爬行速度 */
#define CFG_POS_CREEP_SPD_RAD_S     0.020f

/* ---------------- 编码器对齐（零位辨识） ---------------- */
#define CFG_ALIGN_UD_V              1.2f        /* 对齐用 d 轴电压。align 期间不受 iq_lim_a 限制，
                                                 * 电流只由相电阻决定：1.8V 实测约 5.5A(≈10W) 纯铜耗，
                                                 * 降到 1.2V 约 3.7A；若转子锁不住再按 1.35/1.5 逐步回加 */
#define CFG_ALIGN_HOLD_TICKS        8000U       /* 对齐保持快环拍数 ≈ 0.8s */
#define CFG_ALIGN_SAMPLE_TICKS      512U        /* 末段采样窗口 */

/* ---------------- 保护阈值 ---------------- */
#define CFG_PROT_PHASE_OC_A         10.0f        /* 给 5A 电流命令保留瞬态超调余量，仍低于 ±10.3A 量程 */
#define CFG_PROT_VBUS_OV_V          65.0f       /* 母线过压（48V+15%≈55V 之上） */
#define CFG_PROT_VBUS_UV_V          12.0f       /* 母线欠压（电机额定范围 12~48V） */

/* ---------------- 电流零偏自校准 ---------------- */
#define CFG_OFFSET_SAMPLES          1024U
#define CFG_OFFSET_DEFAULT_RAW      2048U

/* ---------------- 开环电压模式默认角速度（电角速度 rad/s） ---------------- */
#define CFG_OPEN_SPD_ELEC_RAD_S     200.0f      /* ≈ 136 rpm 转子，新板保守起步 */

/* ---------------- PWM 发波测试默认占空比（故意不相等，便于示波器辨认） ---------------- */
#define CFG_PWM_TEST_DUTY_A         0.30f
#define CFG_PWM_TEST_DUTY_B         0.40f
#define CFG_PWM_TEST_DUTY_C         0.60f

/* ---------------- 编码器观测器内部参数 ---------------- */
#define CFG_ENC_LSB_RAD             (6.28318530718f / 65536.0f)
#define CFG_SPD_ZERO_HOLD_SCALE     4.0f        /* 低速量化抑制系数（5 样本窗下名义死区 ≈7.3rpm 转子） */
#define CFG_SPD_ZERO_HOLD_MIN_RAD_S 0.35f
#define CFG_ENC_RENORM_RAD          (32.0f * 6.28318530718f)  /* 连续角归一化阈值 */

/* ---------------- 遥测 ---------------- */
#define CFG_VOFA_PERIOD_MS          3U          /* 5 路 VOFA，3ms ≈ 333Hz；115200 下每帧约 2.08ms */
#define CFG_VBUS_LPF_HZ             50.0f       /* 1kHz 更新时 alpha≈0.2696 */

#endif /* APP_CONFIG_H */
