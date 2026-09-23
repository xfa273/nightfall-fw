#include "f413_diag.h"
#include "f413_trace_diag.h"
#include "nvm_params.h"
#include "trace.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Real diagnostic entry points + real serializers, in-memory NVM only. */
static unsigned char storage[NVM_AREA_COUNT][4096];
static unsigned char before[NVM_AREA_COUNT][4096];
static unsigned writes, erases, aborts, refusals;

nvm_status_t nvm_get_area_info(nvm_area_t area, nvm_area_info_t* out)
{
  assert(area < NVM_AREA_COUNT && out != NULL);
  *out = (nvm_area_info_t){area, 0, sizeof(storage[0]), 0};
  return NVM_STATUS_OK;
}
nvm_status_t nvm_read(nvm_area_t area, uint32_t off, void* out, size_t len)
{
  assert(area < NVM_AREA_COUNT && off <= sizeof(storage[0]));
  assert(len <= sizeof(storage[0]) - off);
  memcpy(out, storage[area] + off, len);
  return NVM_STATUS_OK;
}
nvm_status_t nvm_write(nvm_area_t area, uint32_t off, const void* data, size_t len)
{
  assert(area != NVM_AREA_IDENTITY && area < NVM_AREA_COUNT);
  assert(off <= sizeof(storage[0]) && len <= sizeof(storage[0]) - off);
  memcpy(storage[area] + off, data, len);
  writes++;
  return NVM_STATUS_OK;
}
nvm_status_t nvm_erase(nvm_area_t area)
{
  assert(area != NVM_AREA_IDENTITY && area < NVM_AREA_COUNT);
  erases++; /* F413 FRAM erase is deliberately a no-op. */
  return NVM_STATUS_OK;
}
bool f413_machine_front_distance_body_centre(void) { return true; }
void sensor_distance_set_warp_fl_3pt(const float x[3], const float y[3])
{ (void)x; (void)y; assert(0); }
void sensor_distance_set_warp_fr_3pt(const float x[3], const float y[3])
{ (void)x; (void)y; assert(0); }
void sensor_distance_set_warp_front_sum_3pt(const float x[3], const float y[3])
{ (void)x; (void)y; assert(0); }
void sensor_distance_clear_warp_fl(void) {}
void sensor_distance_clear_warp_fr(void) {}
void sensor_distance_clear_warp_front_sum(void) {}
void f413_trace_log_auto_abort(void) { aborts++; }
int trace_printf(const char* fmt, ...)
{
  if (strstr(fmt, "[NVM-GUARD] REFUSED") != NULL) refusals++;
  return 0;
}
void trace_write(const char* p, size_t n) { (void)p; (void)n; }
static void fill_sample(nvm_trace_log_record_t* out, uint32_t seq)
{ memset(out, 0, sizeof(*out)); out->seq = seq; }

int main(void)
{
  const bool enabled = NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS != 0;
  nvm_sensor_params_t sensor, loaded;
  nvm_params_sensor_defaults(&sensor);
  sensor.wall_offset_fr = 708; sensor.wall_offset_fl = 681;
  sensor.wall_offset_r = 551; sensor.wall_offset_l = 570;
  memset(storage, 0x5A, sizeof(storage));
  /* Normal calibration save/load must remain usable with the guard locked. */
  assert(nvm_params_sensor_save(&sensor) == HAL_OK);
  assert(nvm_params_sensor_load(&loaded));
  sensor.crc = 0x1D6;
  assert(sizeof(sensor) == 68 && memcmp(&loaded, &sensor, sizeof(sensor)) == 0);
  memcpy(before, storage, sizeof(storage));
  writes = erases = 0;
  f413_trace_diag_config(&(f413_trace_diag_config_t){.fill_sample = fill_sample});

  assert(f413_diag_run_distance_nvm_test() == enabled);
  assert(f413_diag_run_sensor_nvm_test() == enabled);
  assert(f413_diag_run_maze_nvm_test() == enabled);
  assert(f413_diag_run_trace_log_nvm_test() == enabled);
  f413_diag_run_all_nvm_tests();
  f413_trace_diag_run_format_once();
  f413_trace_diag_run_append_sample_once();
  f413_trace_diag_run_selftest_once();
  if (!enabled) {
    assert(refusals == 8 && writes == 0 && erases == 0 && aborts == 0);
    assert(memcmp(storage, before, sizeof(storage)) == 0);
    /* Read-only verification remains read-only even when fixture comparison fails. */
    f413_diag_verify_all_nvm_load_only();
    assert(writes == 0 && erases == 0 && aborts == 0);
    assert(memcmp(storage, before, sizeof(storage)) == 0);
    puts("PASS: default guard refuses all eight destructive entry points; normal calibration works; all NVM bytes preserved");
  } else {
    nvm_trace_log_header_t header;
    assert(refusals == 0 && writes > 0 && erases > 0 && aborts == 2);
    assert(nvm_trace_log_get_header(&header) == NVM_STATUS_OK && header.total_records == 16);
    assert(f413_diag_verify_sensor_nvm_load_only());
    assert(memcmp(storage[NVM_AREA_IDENTITY], before[NVM_AREA_IDENTITY], sizeof(storage[0])) == 0);
    puts("PASS: explicit maintenance opt-in reaches writes in host memory only");
  }
  return 0;
}
