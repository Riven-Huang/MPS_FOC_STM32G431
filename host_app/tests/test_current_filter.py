"""Execute real current feedback/protection functions with only hardware mocked."""
from pathlib import Path
import shutil
import subprocess
import pytest


def extract_function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def test_current_feedback_filter_and_unfiltered_protection(tmp_path):
    root = Path(__file__).resolve().parents[2]
    gcc = shutil.which('gcc') or 'C:/Program Files (x86)/Dev-Cpp/MinGW64/bin/gcc.exe'
    if not Path(gcc).is_file():
        pytest.skip('Native GCC required')
    source = (root / 'program/App/motor_ctrl.c').read_text(encoding='utf-8')
    declarations = r'''
#include <assert.h>
#include <math.h>
#include "motor.h"
#include "app_config.h"
#include "filter.h"
#include "units.h"
volatile motor_cmd_t g_cmd;
volatile motor_fb_t g_fb;
foc_core_t g_foc;
static filter_lpf_f32_t s_id_lpf, s_iq_lpf;
static float s_cur_lpf_hz_cache = -1;
static uint16_t s_ofs_a=2048, s_ofs_b=2048, s_ofs_c=2048;
static motor_state_id_t s_state = MOTOR_STATE_RUN;
static motor_mode_t s_mode_active = MOTOR_MODE_CURRENT;
static struct { unsigned data_valid; } s_enc = {1};
static unsigned powered = 1;
static unsigned board_power_enabled(void) { return powered; }
static unsigned board_drv_fault(void) { return 0; }
static unsigned ctrl_mode_needs_encoder(motor_mode_t mode) { (void)mode; return 1; }
static float board_current_a(uint16_t raw, uint16_t offset) {
    return ((float)raw-offset)*CFG_ADC_REF_V/CFG_ADC_FULL_SCALE/(CFG_CUR_GAIN*CFG_SHUNT_OHM);
}
'''
    functions = '\n'.join(extract_function(source, signature) for signature in (
        'static float ctrl_clamp(', 'static float ctrl_lpf_alpha(',
        'static void ctrl_current_feedback(', 'static uint16_t ctrl_check_fast_faults('))
    checks = r'''
int main(void) {
    float raw_d, raw_q, alpha, previous;
    unsigned k;
    g_fb.offset_ready=1;
    g_cmd.prot_oc_a=16;
    g_cmd.cur_lpf_hz=1500;
    g_foc.cos_theta=1;
    ctrl_current_feedback(2048,2048,2048);
    ctrl_current_feedback(3048,2048,1048);
    raw_d=board_current_a(3048,2048);
    raw_q=raw_d/sqrtf(3);
    alpha=1-expf(-2*3.14159265358979323846f*1500/10000);
    assert(fabsf(g_fb.id_a-alpha*raw_d)<1e-5f);
    assert(fabsf(g_fb.iq_a-alpha*raw_q)<1e-5f);
    for(k=0;k<30;++k) {
        previous=g_fb.id_a;
        ctrl_current_feedback(3048,2048,1048);
        assert(g_fb.id_a>=previous && g_fb.id_a<=raw_d+1e-5f);
    }
    assert(fabsf(g_fb.id_a-raw_d)<1e-4f);
    /* Bypass follows both signed input changes without lag. */
    g_cmd.cur_lpf_hz=0;
    ctrl_current_feedback(1048,2048,3048);
    assert(fabsf(g_fb.id_a+raw_d)<1e-5f);
    ctrl_current_feedback(3048,2048,1048);
    assert(fabsf(g_fb.id_a-raw_d)<1e-5f);
    /* Disabled power clears history; first enabled sample is unattenuated. */
    powered=0;
    ctrl_current_feedback(2048,2048,2048);
    assert(g_fb.id_a==0 && !s_id_lpf.initialized);
    powered=1; g_cmd.cur_lpf_hz=1500;
    ctrl_current_feedback(1048,2048,3048);
    assert(fabsf(g_fb.id_a+raw_d)<1e-5f);
    /* A very slow filter must not hide a one-sample 18 A overcurrent. */
    g_cmd.cur_lpf_hz=10;
    ctrl_current_feedback(2048,2048,2048);
    ctrl_current_feedback(4048,2048,48);
    assert(fabsf(g_fb.id_a)<1);
    assert(g_fb.ia_a>16 && g_fb.ic_a < -16);
    assert(ctrl_check_fast_faults() & FAULT_BIT_OVERCURRENT);
    g_fb.offset_ready=0;
    ctrl_current_feedback(2048,2048,2048);
    assert(!s_id_lpf.initialized && !s_iq_lpf.initialized);
    return 0;
}
'''
    cfile = tmp_path / 'current_feedback.c'
    cfile.write_text(declarations + functions + checks, encoding='utf-8')
    exe = tmp_path / 'current_feedback.exe'
    subprocess.run([gcc, '-std=c99', '-Wall', '-Wextra', '-I'+str(root/'program/App'),
                    str(cfile), str(root/'program/App/filter.c'), str(root/'program/App/foc_core.c'),
                    '-o', str(exe), '-lm'], check=True)
    subprocess.run([str(exe)], check=True, capture_output=True, text=True)
