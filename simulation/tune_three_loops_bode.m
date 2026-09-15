%[text] # 三环 Bode 参数整定（电流环 → 速度环 → 位置环）
%[text] 本脚本按 `video/三环控制框图.png` 的串级结构逐级建模：位置控制器 $G_\\theta(s)$、速度控制器 $G_\\omega(s)$、电流控制器 $G_c(s)$ 依次串接，中间串控制延迟 $G_d(s)$、电流对象 $1/(Ls+R)$、转矩系数 $K_t$、机械对象 $1/(Js+B)$ 与位置积分 $1/s$；三条反馈支路各带一个“检测时延与滤波”环节 $H_i(s)$、$H_\\omega(s)$、$H_\\theta(s)$。
%[text] 整定顺序由内到外：先把电流环闭环 $T_i(s)$ 当作速度环对象的一部分，再把速度环闭环 $T_\\omega(s)$ 当作位置环对象的一部分。每一级都在开环 Bode 上取 $|L(j\\omega_c)|=1$，检查相位裕度与增益裕度，再用闭环阶跃校核超调与调节时间。
%[text] 本脚本独立完成参数分析与绘图；参数只在第 1 节手动设置，不读取或修改固件文件。增益公式保留用于模型分析。
%[text:table]
%[text] | 框图环节 | 脚本中的模型 | 代码对应 |
%[text] | --- | --- | --- |
%[text] | 电流控制器 $G_c(s)$ | $K_{p,i}+K_{i,i}/s$ | `g_cmd.cur_kp` / `cur_ki` |
%[text] | 电流对象 | $1/(Ls+R)$ | `CFG_CUR_EQ_R_OHM` / `CFG_CUR_EQ_L_H` |
%[text] | 控制延迟 $G_d(s)$ | $e^{-T_d s}$ | PWM 更新率 + 采样计算 |
%[text] | 电流反馈 $H_i(s)$ | $1$（代码未加低通） | `ctrl_current_feedback()` |
%[text] | 速度控制器 $G_\\omega(s)$ | $K_{p,\\omega}+K_{i,\\omega}/s$，电角速度域 | `g_cmd.spd_kp` / `spd_ki` |
%[text] | 机械对象 | $K_t/(Js+B)$ | `CFG_MOTOR_KT_NM_A` / `CFG_MOTOR_INERTIA_KGM2` |
%[text] | 速度反馈 $H_\\omega(s)$ | $e^{-T_d s}/(1+s/\\omega_f)$ | 测速窗 + `CFG_SPD_LPF_HZ_DEFAULT` |
%[text] | 位置控制器 $G_\\theta(s)$ | $K_{p,\\theta}+K_{d,\\theta}s$（$K_d$ 加在反馈支路） | `g_cmd.pos_kp` / `pos_kd` |
%[text] | 位置积分 | $1/s$ | 输出轴角度 |
%[text] | 位置反馈 $H_\\theta(s)$ | $e^{-T_d s}$ | 位置环 200 Hz + 编码器 |
%[text:table]
%%
%[text] ## 1. 独立仿真参数
%[text] 下列参数为脚本内的仿真基准值。修改固件不会自动改变本脚本，运行本脚本也不会改变固件；需要对照实机时请手动核对参数。
cfg_names = {'CFG_MOTOR_POLE_PAIRS', 'CFG_MOTOR_GEAR_RATIO', 'CFG_MOTOR_KT_NM_A', 'CFG_MOTOR_INERTIA_KGM2', ...
             'CFG_CUR_EQ_R_OHM', 'CFG_CUR_EQ_L_H', 'CFG_FAST_LOOP_HZ', 'CFG_SPD_WINDOW_SAMPLES', ...
             'CFG_POS_LOOP_HZ', 'CFG_SPD_LPF_HZ_DEFAULT', 'CFG_CUR_BW_HZ_DEFAULT', 'CFG_SPD_BW_HZ_DEFAULT', ...
             'CFG_SPD_DAMPING', 'CFG_POS_BW_HZ_DEFAULT', 'CFG_POS_KI_DEFAULT', 'CFG_POS_KD_DEFAULT'};
cfg_defs = [14, 8, 0.60, 3.0e-4, 0.725, 4.10e-4, 10000, 5, 200, 300, 500, 30, 1.0, 5, 0, 0.3];
cfg_vals = cfg_defs;
cfg = struct();
for k = 1:numel(cfg_names)
    cfg.(cfg_names{k}) = cfg_vals(k);
end
disp("参数来源：本脚本第 1 节的独立仿真参数")
pole_pairs = cfg.CFG_MOTOR_POLE_PAIRS;
Kt = cfg.CFG_MOTOR_KT_NM_A;
J = cfg.CFG_MOTOR_INERTIA_KGM2;
Bv = 0;
R_eq = cfg.CFG_CUR_EQ_R_OHM;
L_eq = cfg.CFG_CUR_EQ_L_H;
f_fast = cfg.CFG_FAST_LOOP_HZ;
f_spd = f_fast/cfg.CFG_SPD_WINDOW_SAMPLES;
f_pos = cfg.CFG_POS_LOOP_HZ;
f_carrier = 20000;
rcr_tim1 = 3;
f_upd = f_carrier/(rcr_tim1 + 1);
%[text] 三环的目标裕度在这里给定。电流环是内层，优先保证它足够快，所以只要求最低裕度；速度环和位置环按目标裕度取可用带宽的最大值。
pm_cur_min = 45;
pm_target = 50;
gm_target = 6;
%[text] 延迟预算采用本脚本设定的节拍：TIM1 中心对齐 20 kHz、`RCR=3` 使占空比按 5 kHz 更新，电流环 10 kHz，速度环 2 kHz，位置环 200 Hz。
T_zoh_cur = 1/(2*f_upd);
T_adc = 0.15e-3;
Td_cur = T_zoh_cur + T_adc;
T_spi = 0.1e-3;
T_win = (cfg.CFG_SPD_WINDOW_SAMPLES/2)/f_fast;
T_zoh_spd = 1/(2*f_spd);
Td_spd = T_spi + T_win + T_zoh_spd;
T_zoh_pos = 1/(2*f_pos);
T_enc = 0.1e-3;
Td_pos = T_zoh_pos + T_enc;
f_spd_lpf = cfg.CFG_SPD_LPF_HZ_DEFAULT;
T_lpf_eq = 1/(2*pi*f_spd_lpf);
delay_stage = ["PWM 更新 ZOH"; "ADC 采样与计算"; "电流反馈合计"; "SPI 样本龄"; "5 点测速窗群延迟"; ...
               "速度环 ZOH"; "测速低通等效群延迟"; "速度反馈合计"; "位置环 ZOH"; "编码器样本龄"; "位置反馈合计"];
delay_ms = 1000*[T_zoh_cur; T_adc; Td_cur; T_spi; T_win; T_zoh_spd; T_lpf_eq; Td_spd + T_lpf_eq; T_zoh_pos; T_enc; Td_pos];
disp(table(delay_stage, delay_ms, 'VariableNames', {'stage', 'delay_ms'}))
%%
%[text] ## 2. 与代码一致的增益公式
%[text] 电流环按模最佳整定，PI 零点抵消电气极点；速度环按二阶极点配置，增益先落在机械 rad/s 域，再除以极对数换算到代码使用的电角速度域；位置环对象含积分环节，比例增益即可定带宽。
bw_cur_code = cfg.CFG_CUR_BW_HZ_DEFAULT;
bw_spd_code = cfg.CFG_SPD_BW_HZ_DEFAULT;
bw_pos_code = cfg.CFG_POS_BW_HZ_DEFAULT;
zeta_spd = cfg.CFG_SPD_DAMPING;
Ki_pos = cfg.CFG_POS_KI_DEFAULT;
Kd_pos_code = cfg.CFG_POS_KD_DEFAULT;
cur_kp = @(bw) 2*pi*bw*L_eq;
cur_ki = @(bw) 2*pi*bw*R_eq;
spd_kp = @(bw) 2*zeta_spd*2*pi*bw*J/Kt/pole_pairs;
spd_ki = @(bw) (2*pi*bw)^2*J/Kt/pole_pairs;
pos_kp = @(bw) 2*pi*bw;
%[text] 用上面的公式计算仿真基准增益，并与脚本内保存的参考数值对照。参考数值不会从固件更新。
chk_gain = ["cur_kp"; "cur_ki"; "spd_kp"; "spd_ki"; "pos_kp"];
chk_doc = [1.288; 2277.65; 0.01346; 1.26895; 31.42];
chk_calc = [cur_kp(bw_cur_code); cur_ki(bw_cur_code); spd_kp(bw_spd_code); spd_ki(bw_spd_code); pos_kp(bw_pos_code)];
disp(table(chk_gain, chk_doc, chk_calc, 'VariableNames', {'gain', 'code_comment', 'script_calc'}))
%[text] 位置环的 $K_d$ 作用在反馈支路（对测速阻尼），它的量纲是时间常数：开环在 $K_i=0$ 时约等于 $(K_{p,\\theta}+K_{d,\\theta}s)/s$，零点落在 $\\omega_z = K_{p,\\theta}/K_{d,\\theta}$。取零点频率为交越频率的若干倍即可。
pos_zero_ratio = 3;
Kd_pos_rec = 1/pos_zero_ratio;
%%
%[text] ## 3. 框图模型
%[text] 三个反馈环节里只有测速支路带低通（`CFG_SPD_LPF_HZ_DEFAULT`），电流与位置支路在代码中没有额外的低通，因此 $H_i$ 取 1、$H_\\theta$ 只含延迟。
s = tf('s');
M_i = 1;
M_w = tf(1, 1, 'InputDelay', Td_spd)/(1 + s/(2*pi*f_spd_lpf));
M_th = tf(1, 1, 'InputDelay', Td_pos);
P_i = tf(1, 1, 'InputDelay', Td_cur)/(L_eq*s + R_eq);
P_w = Kt/(J*s + Bv);
C_i = @(bw) cur_kp(bw) + cur_ki(bw)/s;
C_w = @(bw) pole_pairs*(spd_kp(bw) + spd_ki(bw)/s);
%[text] 内环闭环用实际量（不是测量量）参与外层建模：$T_i = C_iP_i/(1+C_iP_iH_i)$，$T_\\omega = C_\\omega P_\\omega/(1+C_\\omega P_\\omega H_\\omega)$。
Ti_code = feedback(C_i(bw_cur_code)*P_i, M_i);
Tw_code = feedback(C_w(bw_spd_code)*Ti_code*P_w, M_w);
L_cur_code = C_i(bw_cur_code)*P_i*M_i;
L_spd_code = C_w(bw_spd_code)*Ti_code*P_w*M_w;
P_th_code = Tw_code/s;
L_pos_code = ((pos_kp(bw_pos_code) + Ki_pos/s)*M_th + Kd_pos_code*s*M_w)*P_th_code;
%%
%[text] ## 4. 电流环：现用值与可用带宽
%[text] 电流环的 PI 零点抵消电气极点后，开环退化成纯积分加延迟 $L(s) = \\omega_c e^{-T_d s}/s$，相位裕度只剩 $90^\\circ - \\omega_c T_d$，所以带宽完全由延迟决定。这也是为什么电流环带宽提不上去时，要先在 CubeMX 里减小 `TIM1` 的 `RCR`。
[gm_cur_code, pm_cur_code, ~, wc_cur_code] = margin(L_cur_code);
bw_grid_cur = logspace(log10(20), log10(2000), 64);
pm_cur_curve = nan(size(bw_grid_cur));
for k = 1:numel(bw_grid_cur)
    [~, pm_cur_curve(k)] = margin(C_i(bw_grid_cur(k))*P_i*M_i);
end
pm_req = [45 50 60];
%[text] 相位裕度随带宽单调下降，所以把曲线反过来插值就能直接读出“某个裕度对应的带宽”，避免反复迭代。
bw_cur_req = bwFromPM(bw_grid_cur, pm_cur_curve, pm_req);
bw_cur_pm60 = bw_cur_req(pm_req == 60);
if (pm_cur_code >= pm_cur_min - 0.1) && (20*log10(gm_cur_code) >= gm_target)
    bw_cur_rec = bw_cur_code;
else
    bw_cur_rec = bw_cur_req(pm_req == pm_cur_min);
end
L_cur_rec = C_i(bw_cur_rec)*P_i*M_i;
L_cur_pm60 = C_i(bw_cur_pm60)*P_i*M_i;
[gm_cur_rec, pm_cur_rec, ~, wc_cur_rec] = margin(L_cur_rec);
[gm_cur_pm60, pm_cur_pm60, ~, wc_cur_pm60] = margin(L_cur_pm60);
cur_row = ["电流环"; "电流环"; "电流环"];
cur_pm_req = pm_req(:);
cur_bw = bw_cur_req(:);
disp(table(cur_row, cur_pm_req, cur_bw, 'VariableNames', {'loop', 'pm_target_deg', 'bw_hz'}))
bode(L_cur_code, L_cur_pm60, logspace(log10(2*pi*5), log10(2*pi*20000), 1500))
legend({sprintf('仿真基准 %.0f Hz (PM %.1f°)', bw_cur_code, pm_cur_code), ...
        sprintf('PM 60° 方案 %.0f Hz (PM %.1f°)', bw_cur_pm60, pm_cur_pm60)}, 'Location', 'southwest')
grid on
title(sprintf('电流环开环 Bode：交越 %.0f Hz / %.0f Hz，增益裕度 %.1f dB / %.1f dB', ...
      wc_cur_code/(2*pi), wc_cur_pm60/(2*pi), 20*log10(gm_cur_code), 20*log10(gm_cur_pm60)))
%%
%[text] ## 5. 电流环：闭环阶跃与带宽取舍
%[text] 500 Hz 对应 45° 相位裕度和 6 dB 增益裕度，是延迟受限下的典型折中；要 60° 就得降到约 330 Hz，代价是内层变慢，速度环可用带宽随之下降。所以脚本对电流环默认保留仿真基准，只在需要更大裕度时才改用 60° 方案。
Ti_rec = feedback(C_i(bw_cur_rec)*P_i, M_i);
Ti_pm60 = feedback(C_i(bw_cur_pm60)*P_i, M_i);
step(Ti_code, Ti_pm60, 0:2e-5:3e-3)
legend({'仿真基准 (PM 45°)', 'PM 60° 方案'}, 'Location', 'southeast')
grid on
title('电流环闭环阶跃（电流给定 1 A）')
cur_step = ["仿真基准 (PM 45°)"; "PM 60° 方案"];
cur_bw_show = [bw_cur_code; bw_cur_pm60];
cur_pm_show = [pm_cur_code; pm_cur_pm60];
cur_gm_show = 20*log10([gm_cur_code; gm_cur_pm60]);
cur_os = [stepinfo(Ti_code).Overshoot; stepinfo(Ti_pm60).Overshoot];
cur_ts = [stepinfo(Ti_code).SettlingTime; stepinfo(Ti_pm60).SettlingTime];
disp(table(cur_step, cur_bw_show, cur_pm_show, cur_gm_show, cur_os, cur_ts, ...
     'VariableNames', {'case_name', 'bw_hz', 'pm_deg', 'gm_db', 'overshoot_pct', 'settle_s'}))
%%
%[text] ## 6. 速度环：把电流环闭环当作对象
%[text] 速度环对象 = 电流环闭环 $T_i(s)$、转矩系数 $K_t$ 与机械对象 $1/(Js+B)$ 的串联，反馈支路是 $H_\\omega(s)$。代码的极点配置只考虑 $K_t/(Js)$，没有计入 1.1 ms 量级的测速延迟，所以实际相位裕度会比设计值低。
bw_grid_spd = logspace(log10(5), log10(200), 64);
pm_spd_curve = nan(size(bw_grid_spd));
for k = 1:numel(bw_grid_spd)
    [~, pm_spd_curve(k)] = margin(C_w(bw_grid_spd(k))*Ti_rec*P_w*M_w);
end
bw_spd_req = bwFromPM(bw_grid_spd, pm_spd_curve, pm_req);
spd_row = repmat("速度环", size(pm_req(:)));
disp(table(spd_row, pm_req(:), bw_spd_req(:), 'VariableNames', {'loop', 'pm_target_deg', 'bw_hz'}))
bw_spd_rec = bw_spd_req(pm_req == pm_target);
L_spd_rec = C_w(bw_spd_rec)*Ti_rec*P_w*M_w;
[gm_spd_code, pm_spd_code, ~, wc_spd_code] = margin(L_spd_code);
[gm_spd_rec, pm_spd_rec, ~, wc_spd_rec] = margin(L_spd_rec);
bode(L_spd_code, L_spd_rec, logspace(log10(2*pi*0.5), log10(2*pi*2000), 1500))
legend({sprintf('仿真基准 %.0f Hz (PM %.1f°)', bw_spd_code, pm_spd_code), ...
        sprintf('按 PM %d° 建议 %.1f Hz (PM %.1f°)', pm_target, bw_spd_rec, pm_spd_rec)}, 'Location', 'southwest')
grid on
title(sprintf('速度环开环 Bode：交越 %.1f Hz / %.1f Hz', wc_spd_code/(2*pi), wc_spd_rec/(2*pi)))
%%
%[text] ## 7. 速度环：闭环阶跃与结论
%[text] 速度环交越频率约为设计带宽的 2 倍（二阶对象的特性），因此 30 Hz 设计值实际交越在 60 Hz 附近，31% 的超调主要来自测速延迟吃掉的那部分相位裕度。降低带宽、提高相位裕度可以换来更小的超调，代价是响应变慢。
Tw_rec = feedback(C_w(bw_spd_rec)*Ti_rec*P_w, M_w);
Twm_code = feedback(L_spd_code, 1);
Twm_rec = feedback(L_spd_rec, 1);
step(Twm_code, Twm_rec, 0:5e-4:0.15)
legend({'仿真基准 (PM 44.5°)', 'PM 50° 建议值'}, 'Location', 'southeast')
grid on
title('速度环闭环阶跃（转子速度给定，测量值）')
spd_case = ["仿真基准 (PM 44.5°)"; "PM 50° 建议值"];
spd_bw_show = [bw_spd_code; bw_spd_rec];
spd_pm_show = [pm_spd_code; pm_spd_rec];
spd_gm_show = 20*log10([gm_spd_code; gm_spd_rec]);
spd_os = [stepinfo(Twm_code).Overshoot; stepinfo(Twm_rec).Overshoot];
spd_ts = [stepinfo(Twm_code).SettlingTime; stepinfo(Twm_rec).SettlingTime];
disp(table(spd_case, spd_bw_show, spd_pm_show, spd_gm_show, spd_os, spd_ts, ...
     'VariableNames', {'case_name', 'bw_hz', 'pm_deg', 'gm_db', 'overshoot_pct', 'settle_s'}))
%%
%[text] ## 8. 位置环：把速度环闭环再套一层
%[text] 位置环对象 = 速度环闭环 × 位置积分 $1/s$，控制器是 $K_{p,\\theta}+K_{d,\\theta}s$，其中 $K_d$ 按代码的做法加在反馈支路。位置环运行在 200 Hz，ZOH 延迟就有 2.5 ms，但速度环闭环在几十 Hz 以内近似为常数，所以位置环带宽的实际限制不是稳定性，而是串级比例：外环交越频率一般不超过内环交越频率的 1/4，否则外环会追着内环的动态跑。
bw_grid_pos = logspace(log10(0.5), log10(100), 64);
pm_pos_curve = nan(size(bw_grid_pos));
for k = 1:numel(bw_grid_pos)
    L_tmp = ((pos_kp(bw_grid_pos(k)) + Ki_pos/s)*M_th + Kd_pos_rec*s*M_w)*(Tw_rec/s);
    [~, pm_pos_curve(k)] = margin(L_tmp);
end
bw_pos_req = bwFromPM(bw_grid_pos, pm_pos_curve, pm_req);
pos_row = repmat("位置环", size(pm_req(:)));
disp(table(pos_row, pm_req(:), bw_pos_req(:), 'VariableNames', {'loop', 'pm_target_deg', 'bw_hz'}))
bw_pos_cap = wc_spd_rec/(2*pi)/4;
bw_pos_rec = min(bw_pos_req(pm_req == pm_target), bw_pos_cap);
L_pos_rec = ((pos_kp(bw_pos_rec) + Ki_pos/s)*M_th + Kd_pos_rec*s*M_w)*(Tw_rec/s);
[gm_pos_code, pm_pos_code, ~, wc_pos_code] = margin(L_pos_code);
[gm_pos_rec, pm_pos_rec, ~, wc_pos_rec] = margin(L_pos_rec);
bode(L_pos_code, L_pos_rec, logspace(log10(2*pi*0.05), log10(2*pi*200), 1500))
legend({sprintf('仿真基准 %.0f Hz (PM %.1f°)', bw_pos_code, pm_pos_code), ...
        sprintf('建议 %.1f Hz (PM %.1f°)', bw_pos_rec, pm_pos_rec)}, 'Location', 'southwest')
grid on
title(sprintf('位置环开环 Bode：交越 %.2f Hz / %.2f Hz', wc_pos_code/(2*pi), wc_pos_rec/(2*pi)))
%%
%[text] ## 9. 位置环：闭环位置阶跃
%[text] 位置环对象自带积分环节，$K_i=0$ 也能消除阶跃位置的稳态误差；加 $K_i$ 反而容易和减速箱静摩擦叠加出低频振荡，所以代码里 `CFG_POS_KI_DEFAULT = 0` 是合理默认值。这里用 90° 阶跃校核：$K_d$ 提供阻尼，降低回正时的超调与抖动。
T_th_code = (pos_kp(bw_pos_code) + Ki_pos/s)*(Tw_code/s)/(1 + L_pos_code);
T_th_rec = (pos_kp(bw_pos_rec) + Ki_pos/s)*(Tw_rec/s)/(1 + L_pos_rec);
step(T_th_code, T_th_rec, 0:2e-3:1.2)
legend({sprintf('仿真基准 %.0f Hz', bw_pos_code), sprintf('建议 %.1f Hz', bw_pos_rec)}, 'Location', 'southeast')
grid on
title('位置环闭环阶跃（输出轴 1 rad，即约 57.3°）')
pos_case = [string(sprintf('仿真基准 %.0f Hz', bw_pos_code)); string(sprintf('建议 %.1f Hz', bw_pos_rec))];
pos_bw_show = [bw_pos_code; bw_pos_rec];
pos_pm_show = [pm_pos_code; pm_pos_rec];
pos_gm_show = 20*log10([gm_pos_code; gm_pos_rec]);
pos_os = [stepinfo(T_th_code).Overshoot; stepinfo(T_th_rec).Overshoot];
pos_ts = [stepinfo(T_th_code).SettlingTime; stepinfo(T_th_rec).SettlingTime];
disp(table(pos_case, pos_bw_show, pos_pm_show, pos_gm_show, pos_os, pos_ts, ...
     'VariableNames', {'case_name', 'bw_hz', 'pm_deg', 'gm_db', 'overshoot_pct', 'settle_s'}))
%[text] 位置环的两条限制要一起看：一是相位裕度，二是速度环交越频率。若把位置带宽提到速度环交越频率的 1/4 以上，外环会开始“追着内环的延迟跑”，阶跃响应出现振荡；另外 `CFG_POS_HOLD_*` 与 `CFG_POS_CREEP_*` 是抑制静摩擦抖动的非线性逻辑，本脚本的线性模型不包含它们。
%%
%[text] ## 10. 带宽—相位裕度曲线
%[text] 三条曲线把“带宽换裕度”的代价放在同一张图上：电流环斜率最陡（纯延迟），速度环次之（电流环闭环 + 测速延迟），位置环在 15 Hz 以内都很平缓，说明现在的 5 Hz 有很大提升空间。
semilogx(bw_grid_cur, pm_cur_curve, bw_grid_spd, pm_spd_curve, bw_grid_pos, pm_pos_curve, 'LineWidth', 1.2)
hold on
yline(pm_cur_min, '--', sprintf('电流环下限 %d°', pm_cur_min))
yline(pm_target, ':', sprintf('速度/位置目标 %d°', pm_target))
hold off
grid on
xlabel('设计带宽 (Hz)')
ylabel('相位裕度 (deg)')
legend({'电流环', '速度环', '位置环'}, 'Location', 'northeast')
title('相位裕度随设计带宽的变化')
%%
%[text] ## 11. 仿真结果汇总
%[text] 下表仅显示仿真基准与建议方案的带宽、裕度和增益，不生成固件配置或写入任何文件。
loop_name = ["电流环"; "速度环"; "位置环"];
bw_code = [bw_cur_code; bw_spd_code; bw_pos_code];
bw_rec = [bw_cur_rec; bw_spd_rec; bw_pos_rec];
pm_code = [pm_cur_code; pm_spd_code; pm_pos_code];
pm_rec = [pm_cur_rec; pm_spd_rec; pm_pos_rec];
gm_code = 20*log10([gm_cur_code; gm_spd_code; gm_pos_code]);
gm_rec = 20*log10([gm_cur_rec; gm_spd_rec; gm_pos_rec]);
disp(table(loop_name, bw_code, pm_code, gm_code, bw_rec, pm_rec, gm_rec, ...
     'VariableNames', {'loop', 'bw_code_hz', 'pm_code_deg', 'gm_code_db', 'bw_suggest_hz', 'pm_suggest_deg', 'gm_suggest_db'}))
Kp_code = [cur_kp(bw_cur_code); spd_kp(bw_spd_code); pos_kp(bw_pos_code)];
Ki_code = [cur_ki(bw_cur_code); spd_ki(bw_spd_code); Ki_pos];
Kp_rec = [cur_kp(bw_cur_rec); spd_kp(bw_spd_rec); pos_kp(bw_pos_rec)];
Ki_rec = [cur_ki(bw_cur_rec); spd_ki(bw_spd_rec); Ki_pos];
Kd_rec = [0; 0; Kd_pos_rec];
disp(table(loop_name, Kp_code, Ki_code, Kp_rec, Ki_rec, Kd_rec, ...
     'VariableNames', {'loop', 'kp_code', 'ki_code', 'kp_suggest', 'ki_suggest', 'kd_suggest'}))
%[text] 使用时按由内到外的顺序改，每改一级都在硬件上复测一次阶跃再进入下一级。位置环建议分两步：先取 8 Hz 复测，确认没有重新出现静摩擦抖动，再提到模型上限。
%[text] 三条限制需要在硬件上复核：一是 `g_cmd.iq_lim_a` 与 $0.577V_{bus}$ 电压矢量限幅，本脚本是线性模型，大阶跃时实际响应会比模型慢；二是 `CFG_POS_HOLD_*` 与 `CFG_POS_CREEP_*` 的静摩擦抑制逻辑是非线性的，位置环提带宽后要重点观察目标附近的低频抖动；三是 `CFG_MOTOR_INERTIA_KGM2` 是估计值，它直接决定速度环 $K_p$，实测偏软就减小、偏硬就增大。
%%
%[text] ## 12. 离散化校核
%[text] 连续域设计的最后一关是离散实现：把三个闭环按各自的实际执行节拍（电流 10 kHz、速度 2 kHz、位置 200 Hz）用零阶保持离散化，与连续响应对比。离散化的延迟近似会引入额外误差，所以这里只用来确认“离散后依然稳定、响应形状一致”。
warning('off', 'Control:transformation:C2dApproximate');
Ti_z = c2d(feedback(C_i(bw_cur_rec)*P_i, M_i), 1/f_fast, 'zoh');
Tw_z = c2d(feedback(C_w(bw_spd_rec)*P_w, M_w), 1/f_spd, 'zoh');
Tth_z = c2d(T_th_rec, 1/f_pos, 'zoh');
warning('on', 'Control:transformation:C2dApproximate');
tiledlayout(1, 3)
nexttile
step(Ti_rec, Ti_z, 0:1/f_fast:2e-3)
legend({'连续', '离散'}, 'Location', 'southeast')
grid on
title('电流环 10 kHz')
nexttile
step(Tw_rec, Tw_z, 0:1/f_spd:0.15)
legend({'连续', '离散'}, 'Location', 'southeast')
grid on
title('速度环 2 kHz')
nexttile
step(T_th_rec, Tth_z, 0:1/f_pos:1.2)
legend({'连续', '离散'}, 'Location', 'southeast')
grid on
title('位置环 200 Hz')
function bw = bwFromPM(grid, pm_curve, pm_req)
% 由“相位裕度—带宽”曲线反查给定裕度对应的带宽（三个环的曲线都随带宽单调下降）。
ok = ~isnan(pm_curve);
pm_ok = pm_curve(ok);
bw_ok = grid(ok);
[pm_u, iu] = unique(pm_ok);
bw = interp1(pm_u, bw_ok(iu), pm_req, 'linear', NaN);
bw = reshape(bw, size(pm_req));
end
%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
