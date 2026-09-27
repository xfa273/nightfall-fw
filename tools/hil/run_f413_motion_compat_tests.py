#!/usr/bin/env python3
"""Differential tests against immutable pre-r3 and pre-isolation production code.

Runs only on the host. The historical source is compiled, not reimplemented as
expected arithmetic. Current machine parameters are used on both sides so local
tuning edits do not invalidate the behavior comparison. Requires git history.
"""
import difflib
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/hil_host/motion_compat'
SRC = 'platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/'
BASE = '57a9c3b'
R3_BASE = '8edeea0'
CAPS = '''
#define NIGHTFALL_F413_PATH_VELOCITY_CAP 1500.0f
#define NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP 1000.0f
#define NIGHTFALL_F413_PATH_TURN_VELOCITY_CAP 500.0f
#define NIGHTFALL_F413_PATH_OMEGA_CAP 2200.0f
'''


def historical(rev, path):
    return subprocess.check_output(['git', 'show', f'{rev}:{path}'], cwd=ROOT, text=True)


CONTROL = r'''
#include <assert.h>
#include <stdio.h>
#include "f413_machine.h"
#include COMPAT_SOURCE
TIM_HandleTypeDef htim2, htim3, htim4, htim5;
SPI_HandleTypeDef hspi2;
static bool fan_active;
static unsigned xl_register;
static int16_t gyro_input;
static float acceleration_input;
static unsigned gpio[5];
bool f413_hw_fan_is_running(void) { return fan_active; }
static const f413_hardware_config_t hardware = {
 .encoder_cpr=200, .encoder_sign_l=1, .encoder_sign_r=-1, .tread_mm=34.5f,
 .imu_forward_accel_reg=0x2a, .imu_forward_accel_sign=1,
 .left_forward_in2_high=false, .right_forward_in2_high=true
};
const f413_hardware_config_t* f413_machine_hardware(void) { return &hardware; }
bool f413_machine_has(uint32_t c) { (void)c; return true; }
void HAL_GPIO_WritePin(unsigned p,unsigned n,GPIO_PinState v) { (void)p; gpio[n]=v; }
void HAL_Delay(uint32_t ms) { (void)ms; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*h,unsigned c) { (void)h;(void)c;return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*h,unsigned c) { (void)h;(void)c;return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*h,unsigned c) { (void)h;(void)c;return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*h,uint8_t*tx,uint8_t*rx,uint16_t n,uint32_t t)
{
 (void)h;(void)t; memset(rx,0,n);
 unsigned reg=tx[0]&0x7f;
 if(reg==0x0f) rx[1]=0x6b;
 if(n==3) {
   int16_t v=reg==0x26 ? gyro_input : (int16_t)lrintf(acceleration_input /
       ((xl_register==0x7c ? 0.244f : 0.488f)*9.80665f));
   rx[1]=(uint8_t)v; rx[2]=(uint16_t)v>>8;
 }
 return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*h,uint8_t*tx,uint16_t n,uint32_t t)
{ (void)h;(void)n;(void)t; if(tx[0]==0x10)xl_register=tx[1];return HAL_OK; }
int main(void)
{
 for(unsigned trial=0;trial<12;trial++) {
   acceleration_input=0; gyro_input=0; fan_active=false;
   f413_ctrl_init(); f413_ctrl_start();
   assert(xl_register==(EXPECT_AUG29 ? 0x7c : 0x74));
   const float sign=(trial&1)?-1.0f:1.0f;
   const unsigned kind=trial/2;
   if(kind==0)f413_ctrl_set_velocity_profile(0,sign*331.662f,55);
   if(kind==1)f413_ctrl_set_velocity_profile(sign*331.662f,sign*300,90);
   if(kind==2)f413_ctrl_set_velocity_profile(sign*300,0,45);
   if(kind==3) { f413_ctrl_set_velocity(sign*300); f413_ctrl_start_omega_profile(sign*800,.09f,.04f); }
   if(kind==4)f413_ctrl_set_velocity_profile(0,sign*2400,1000);
   if(kind==5)f413_ctrl_tune_start(F413_CTRL_TUNE_AXIS_VELOCITY,0,F413_CTRL_TUNE_PATTERN_STEP);
   for(unsigned tick=0;tick<700;tick++) {
     int counts=(kind==4?10:1)+(tick%5==0)-(tick%7==0);
     htim3.counter=F413_CTRL_ENCODER_CENTER+(int)sign*counts;
     htim4.counter=F413_CTRL_ENCODER_CENTER-(int)sign*(counts+(tick%13==0));
     acceleration_input=sign*((int)(tick%47)-23)*70.0f;
     gyro_input=(int16_t)((int)(tick%41)-20);
     f413_ctrl_tick();
     printf("%u %u %.6f %.6f %.6f %.6f %.6f %.6f %d %d %u %u %u %u\n",
       trial,tick,f413_ctrl_get_distance(),f413_ctrl_get_target_distance(),
       f413_ctrl_get_real_velocity(),f413_ctrl_get_target_velocity(),
       s_acceleration_interrupt,f413_ctrl_get_target_omega(),
       f413_ctrl_get_motor_out_l(),f413_ctrl_get_motor_out_r(),
       htim2.compare[0],htim2.compare[2],gpio[3],gpio[4]);
   }
   f413_ctrl_stop();
 }
}
'''

SEARCH_MAIN = r'''
int main(void)
{
  const float edges[]={-1, 40, 415, 424, 445, 505};
  for(unsigned lag=0;lag<4;lag++) for(unsigned e=0;e<6;e++) {
    reset();
    SearchRunParams_t params=searchRunParams[0]; params.wall_align_enable=0;
    f413_run_features_t features={.wall_end_correction_enabled=true};
    f413_run_features_set(&features);
    f413_run_session_guard_t guard={0};float v=0;bool acceled=false;
    edge_position=edges[e]<0?INFINITY:edges[e];
    assert(f413_search_step_run_entry_section(2,F413_SEARCH_STEP_TARGET_FULL,&params,&v,&guard)==0);
    for(unsigned cell=1;cell<=8;cell++) {
      advance(cell==1?9:lag*3);
      f413_search_step_motion_detail_t detail={0};
      int status=f413_search_step_run_forward_section(&params,&v,&guard,&acceled,false,false,NULL,&detail);
      printf("edge %u %u %u %d %.6f %.6f %u\n",lag,e,cell,status,position,profile_end,edge_fired);
      if(edge_fired || status)break;
    }
  }
  for(unsigned turn=0;turn<2;turn++) for(unsigned lag=0;lag<3;lag++) {
    reset();float v=300;velocity=v;advance(lag*4);
    f413_run_session_guard_t guard={0};
    int status=f413_search_step_run_smooth_turn(turn?3:1,&searchRunParams[0],&v,&guard);
    printf("turn %u %u %d %.6f %.6f %.6f %.6f\n",turn,lag,status,position,angle,last_turn_entry,last_turn_exit);
  }
  for(unsigned near=0;near<4;near++) {
    reset();wall_align_model=true;
    front_wall_position=F_ALIGN_TARGET_MM+(near==0?5:near==1?-3:near==2?20:0);
    f413_run_session_guard_t guard={0}; f413_search_step_front_match_result_t result;
    int status=f413_search_step_match_front_position(&guard,&result);
    printf("align %u %d %d %u %.6f %.6f %.6f\n",near,status,result.status,
        result.elapsed_ms,position,result.position_error_mm,result.yaw_error_mm);
  }
  return 0;
}
'''



PATH_MAIN = r"""
#include <stdio.h>
#define NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST 1
#include COMPAT_SOURCE
int main(void) {
 const ShortestRunModeParams_t* modes[]={&shortestRunModeParams2,&shortestRunModeParams3,&shortestRunModeParams4,&shortestRunModeParams5,&shortestRunModeParams6,&shortestRunModeParams7};
 const ShortestRunCaseParams_t* cases[]={shortestRunCaseParamsMode2,shortestRunCaseParamsMode3,shortestRunCaseParamsMode4,shortestRunCaseParamsMode5,shortestRunCaseParamsMode6,shortestRunCaseParamsMode7};
 const uint16_t paths[][8]={{203,300,0},{203,501,0},{203,502,0},{203,701,1001,0},{203,701,1001,704,0},{203,701,1001,802,1001,0},{203,901,1001,0},{203,901,1001,904,0},{209,0},{203,701,1001,802,1001,703,202,0},{209,501,209,503,202,0},{205,502,0}};
 for(unsigned m=0;m<6;m++)for(unsigned c=0;c<9;c++)for(unsigned p=0;p<12;p++)for(unsigned test=0;test<2;test++) {
   ShortestRunCaseParams_t selected=cases[m][c];
#if !OLD_R2
   selected=f413_path_run_session_case_params(paths[p],8,modes[m],&selected,test);
#endif
   f413_path_run_prepared_path_t prepared={0};
   const float first=fminf(sqrtf(2.0f*selected.acceleration_straight*DIST_FIRST_SEC),1500.0f);
   f413_path_run_preflight_result_t result=f413_path_run_preflight_prepare(paths[p],8,modes[m],&selected,first,false,test,&prepared);
   printf("path %u %u %u %u %u %u %zu %u %zu",m,c,p,test,result.status,result.legacy_status,result.index,result.code,prepared.count);
   for(size_t a=0;a<prepared.count;a++) {
     const f413_path_run_prepared_linear_t* v=&prepared.actions[a];
     printf(" | %u %u %u %.6f %.6f",v->path_index,v->phase_count,v->execute_plan,v->entry_velocity_mm_s,v->exit_velocity_mm_s);
     for(unsigned i=0;i<v->phase_count;i++)printf(" %.6f %.6f",v->phase_distance_mm[i],v->phase_exit_velocity_mm_s[i]);
   }
   printf("\n");
 }
 return 0;
}
"""

MODE_MAIN = r"""
#include <stdio.h>
#include COMPAT_SOURCE
static void features(const f413_run_features_t* f) {
 printf(" %u %u %u %u %u",f->wall_control_enabled,f->wall_end_correction_enabled,f->front_wall_correction_enabled,f->angle_accum_mode,f->test_mode_run);
}
void f413_mode_shortest_run_case(uint8_t m,uint8_t c) { printf("fallback %u %u\n",m,c); }
void f413_mode_shortest_run_config(const f413_shortest_case_config_t* c) {
 printf("config %u %u %u",c->mode,c->op_case,c->diagonal_time_plan);features(&c->features);printf("\n");
}
void f413_mode_shortest_run_case0_path(const char*l,uint8_t m,uint8_t c,const uint16_t*codes,uint16_t n) {
 (void)l;printf("codes %u %u",m,c);for(unsigned i=0;i<n;i++)printf(" %u",codes[i]);printf("\n");
}
void f413_mode_shortest_run_path_config(const char*l,uint8_t m,uint8_t c,const uint16_t*codes,uint16_t n,const f413_run_features_t*f) {
 features(f);f413_mode_shortest_run_case0_path(l,m,c,codes,n);
}
int main(void) { for(unsigned c=0;c<=10;c++)MODE_RUN(c);for(unsigned s=0;s<=10;s++)MODE_SUB(s); }
"""


def compile_run(name, source, profile, extras=(), flags=()):
    file = OUT / f'{name}.c'
    file.write_text(source)
    binary = OUT / name
    dirs = ['tools/hil/control_stubs', 'tools/hil/nvm_stubs', f'params/{profile}',
            'board/f413', 'nvm', 'common/route', 'platform/trace',
            'platform/stm32f405/Core/Inc', SRC.replace('/Src/', '/Inc/')]
    command = [os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
               '-Wno-unused-function', '-Wno-array-bounds', '-O1', '-g', '-DSTM32F413xx',
               '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
               '-ffunction-sections', '-fdata-sections', *flags,
               *[f'-I{ROOT / d}' for d in dirs], str(file),
               *[str(ROOT / p) for p in extras],
               '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections',
               '-lm', '-o', str(binary)]
    subprocess.run(command, check=True)
    result = subprocess.check_output([str(binary)], text=True, env=dict(os.environ,
        ASAN_OPTIONS='detect_leaks=0:halt_on_error=1', UBSAN_OPTIONS='halt_on_error=1'))
    file.with_suffix('.txt').write_text(result)
    return result


def compare(name, old, new):
    if old != new:
        diff = ''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                                         fromfile='reference', tofile='current'))
        (OUT / f'{name}.diff').write_text(diff)
        raise AssertionError(f'{name} differs; see {OUT / (name + ".diff")}\n{diff[:2200]}')
    print(f'PASS: {name}: {len(old.splitlines())} records identical', flush=True)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for profile, rev in [('f413_preorder', BASE), ('mini_r3_0', R3_BASE)]:
        results = {'control': [], 'search': [], 'path': [], 'modes': []}
        for variant in ['reference', 'current']:
            sources = {}
            for module in ['f413_control.c', 'f413_search_step.c', 'f413_path_run.c'] + [f'f413_mode{m}.c' for m in range(2, 8)]:
                path = ROOT / SRC / module
                if variant == 'reference':
                    path = OUT / f'{profile}_{module}'
                    path.write_text(CAPS + historical(rev, SRC + module))
                sources[module] = path
            results['control'].append(compile_run(f'{profile}_control_{variant}', CONTROL,
                profile, flags=[f'-DCOMPAT_SOURCE="{sources["f413_control.c"]}"',
                                f'-DEXPECT_AUG29={int(profile == "f413_preorder")}']))
            # Reuse the pinned, hardware-free ideal follower. No production policy override.
            fixture = historical(R3_BASE, 'tools/hil/f413_search_distance_tests.c').split('static void close_to')[0]
            fixture = fixture.replace('../../' + SRC + 'f413_search_step.c', str(sources['f413_search_step.c']))
            if variant == 'reference' and rev == BASE:
                fixture = fixture.replace('f413_search_step_reset_distance();', 'f413_ctrl_reset_distance();')
            fixture = '#include "f413_motion_stop.h"\n' + fixture
            results['search'].append(compile_run(f'{profile}_search_{variant}', fixture + SEARCH_MAIN,
                profile, extras=[SRC + 'f413_front_match.c', SRC + 'f413_run_features.c',
                                 f'params/{profile}/search_run_params_split.c']))
            results['path'].append(compile_run(f'{profile}_path_{variant}', PATH_MAIN,
                profile, extras=['common/route/motion_time.c', 'common/route/legacy_path_codec.c',
                                 f'params/{profile}/shortest_run_params_split.c'],
                flags=[f'-DCOMPAT_SOURCE="{sources["f413_path_run.c"]}"',
                       f'-DOLD_R2={int(variant == "reference" and rev == BASE)}']))
            mode_output = []
            for mode in range(2, 8):
                module = f'f413_mode{mode}.c'
                mode_output.append(compile_run(f'{profile}_mode{mode}_{variant}', MODE_MAIN,
                    profile, extras=[f'params/{profile}/shortest_run_params_split.c'],
                    flags=[f'-DCOMPAT_SOURCE="{sources[module]}"',
                           f'-DMODE_RUN=f413_mode{mode}_run_case',
                           f'-DMODE_SUB=f413_mode{mode}_run_case0_sub']))
            results['modes'].append(''.join(mode_output))
        for module in results:
            compare(f'{profile}_{module}', *results[module])


if __name__ == '__main__':
    main()
