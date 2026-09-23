#include "nvm_params.h"
#include "nvm.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint32_t magic, version, length, crc;
  float points[6][3];
  uint32_t reserved[8];
} blob_t;
static blob_t blob;
static unsigned applied, cleared;
static bool body_centre;
bool f413_machine_front_distance_body_centre(void) { return body_centre; }
nvm_status_t nvm_read(nvm_area_t area, uint32_t off, void *out, size_t n)
{
  assert(area == NVM_AREA_DISTANCE_PARAMS && off == 0 && n == sizeof(blob));
  memcpy(out, &blob, n);
  return NVM_STATUS_OK;
}
nvm_status_t nvm_write(nvm_area_t a, uint32_t o, const void *p, size_t n)
{ (void)a; (void)o; (void)p; (void)n; abort(); }
nvm_status_t nvm_erase(nvm_area_t a) { (void)a; abort(); }
void sensor_distance_set_warp_fl_3pt(const float x[3], const float y[3])
{ assert(memcmp(x, blob.points[0], 12) == 0 && memcmp(y, blob.points[1], 12) == 0); applied++; }
void sensor_distance_set_warp_fr_3pt(const float x[3], const float y[3])
{ assert(memcmp(x, blob.points[2], 12) == 0 && memcmp(y, blob.points[3], 12) == 0); applied++; }
void sensor_distance_set_warp_front_sum_3pt(const float x[3], const float y[3])
{ assert(memcmp(x, blob.points[4], 12) == 0 && memcmp(y, blob.points[5], 12) == 0); applied++; }
void sensor_distance_clear_warp_fl(void) { cleared++; }
void sensor_distance_clear_warp_fr(void) { cleared++; }
void sensor_distance_clear_warp_front_sum(void) { cleared++; }
static void seal(void)
{
  blob.magic = 0x44495354;
  blob.version = 0x00010000;
  blob.length = sizeof(blob);
  blob.crc = 0;
  for (size_t i = 16; i < sizeof(blob); i++) blob.crc += ((const unsigned char*)&blob)[i];
}
int main(void)
{
  const float diagnostic[6][3] = {
    {230,420,680}, {180,360,540}, {235,425,685},
    {180,360,540}, {465,845,1365}, {180,360,540}
  };
  memcpy(blob.points, diagnostic, sizeof(diagnostic)); seal();
  blob_t before = blob;
  assert(!nvm_params_distance_load_and_apply());
  assert(applied == 0 && cleared == 3 && memcmp(&blob, &before, sizeof(blob)) == 0);
  /* Normal calibration still passes and reaches every warp without mutation. */
  for (unsigned row = 0; row < 6; row++) {
    blob.points[row][0] = 0; blob.points[row][1] = 26; blob.points[row][2] = 113;
  }
  seal(); before = blob;
  assert(nvm_params_distance_load_and_apply());
  assert(applied == 3 && cleared == 3 && memcmp(&blob, &before, sizeof(blob)) == 0);
  blob.crc++;
  assert(!nvm_params_distance_load_and_apply() && applied == 3);
  seal(); blob.version++;
  assert(!nvm_params_distance_load_and_apply() && applied == 3);
  /* A genuine legacy warp must also not affect the new centre-reference LUT. */
  seal(); before = blob; body_centre = true;
  assert(!nvm_params_distance_load_and_apply());
  assert(applied == 3 && cleared == 6 && memcmp(&blob, &before, sizeof(blob)) == 0);
  puts("PASS: F413 dummy/reference-incompatible warps rejected, legacy calibration preserved, no NVM writes");
  return 0;
}
