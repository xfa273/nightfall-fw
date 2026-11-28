#define SEARCH_REAL_WALL_RUNTIME
#define main search_distance_fixture_main
#include "f413_search_distance_tests.c"
#undef main
void f413_ctrl_set_heading_omega_correction(float v) { (void)v; }
void f413_wall_sensor_get_control_base(uint16_t* l,uint16_t* r,uint16_t* f)
{ if(l)*l=600;if(r)*r=600;if(f)*f=1000; }
static bool real_snapshot(f413_wall_sensor_snapshot_t* out)
{
  *out=wall;out->sample_sequence=tick/2;
  out->r_delta=position<12?600:40;out->l_delta=600;
  return true;
}
int main(void)
{
  for(unsigned disabled=0;disabled<2;disabled++) {
    reset(); const f413_wall_runtime_config_t cfg={real_snapshot,HAL_Delay,HAL_GetTick,0,0,4};
    f413_wall_runtime_config(&cfg);f413_wall_runtime_set_wall_end_thresholds(100,1,100,1);
    f413_run_features_t f=f413_run_features_get();f.wall_end_correction_enabled=!disabled;f413_run_features_set(&f);
    float speed=1000;bool found=false;f413_run_session_guard_t guard={0};
    assert(f413_search_step_drive_wallend_segment(45,1000,&speed,&guard,4,&found,NULL,NULL)==0);
    const bool fast=!disabled && F413_MOTION_ENABLED(F413_MOTION_SHORT_WALL_END_ALL);
    assert(found==!disabled);assert(f413_wall_runtime_wall_end_detected_by_short()==fast);
    close_to(position,disabled?45:fast?14:20);
    if(found)close_to(g_search_distance_endpoint_mm,position);
  }
  puts("PASS: production search + real wall runtime, early detection/rebase, OFF and r2 fallback");
}
