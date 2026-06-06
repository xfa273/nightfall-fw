#include "f413_diag.h"

#include <string.h>

#include "nvm.h"
#include "nvm_params.h"
#include "trace.h"

#define F413_DIAG_TEST_MAZE_CELLS (32U)
#define F413_DIAG_TEST_TRACE_BYTES (32U)
#define F413_DIAG_TEST_TRACE_OFFSET (0x100U)

static void f413_diag_fill_expected_sensor_params(nvm_sensor_params_t* out)
{
  if (out == NULL)
  {
    return;
  }

  nvm_params_sensor_defaults(out);
  out->base_l = 1111U;
  out->base_r = 1222U;
  out->base_f = 1333U;
  out->wall_offset_r = 200U;
  out->wall_offset_l = 210U;
  out->wall_offset_fr = 220U;
  out->wall_offset_fl = 230U;
  out->imu_offset_z = 1.25f;
}

static void f413_diag_fill_expected_maze_cells(uint16_t* out, uint32_t count)
{
  uint32_t i;

  if ((out == NULL) && (count > 0U))
  {
    return;
  }

  for (i = 0U; i < count; i++)
  {
    out[i] = (uint16_t)(0x1000U + i * 3U);
  }
}

static void f413_diag_fill_expected_trace_bytes(uint8_t* out, uint32_t count)
{
  uint32_t i;

  if ((out == NULL) && (count > 0U))
  {
    return;
  }

  for (i = 0U; i < count; i++)
  {
    out[i] = (uint8_t)(0xA0U + (i * 7U));
  }
}

bool f413_diag_run_distance_nvm_test(void)
{
  static const float x_fl[3] = {230.0f, 420.0f, 680.0f};
  static const float y_fl[3] = {180.0f, 360.0f, 540.0f};
  static const float x_fr[3] = {235.0f, 425.0f, 685.0f};
  static const float y_fr[3] = {180.0f, 360.0f, 540.0f};
  static const float x_fsum[3] = {465.0f, 845.0f, 1365.0f};
  static const float y_fsum[3] = {180.0f, 360.0f, 540.0f};

  HAL_StatusTypeDef st = nvm_params_distance_save(x_fl, y_fl, x_fr, y_fr, x_fsum, y_fsum);
  if (st != HAL_OK)
  {
    trace_printf("[NVM-TEST][Distance] save: FAIL (HAL=%d)\r\n", (int)st);
    return false;
  }

  if (!nvm_params_distance_load_and_apply())
  {
    trace_printf("[NVM-TEST][Distance] load_and_apply: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Distance] save/load_and_apply: PASS\r\n");
  return true;
}

bool f413_diag_run_sensor_nvm_test(void)
{
  nvm_sensor_params_t save_blob;
  nvm_sensor_params_t load_blob;
  HAL_StatusTypeDef st;

  f413_diag_fill_expected_sensor_params(&save_blob);

  st = nvm_params_sensor_save(&save_blob);
  if (st != HAL_OK)
  {
    trace_printf("[NVM-TEST][Sensor] save: FAIL (HAL=%d)\r\n", (int)st);
    return false;
  }

  memset(&load_blob, 0, sizeof(load_blob));
  if (!nvm_params_sensor_load(&load_blob))
  {
    trace_printf("[NVM-TEST][Sensor] load: FAIL\r\n");
    return false;
  }

  if ((load_blob.base_l != save_blob.base_l) ||
      (load_blob.base_r != save_blob.base_r) ||
      (load_blob.base_f != save_blob.base_f) ||
      (load_blob.wall_offset_r != save_blob.wall_offset_r) ||
      (load_blob.wall_offset_l != save_blob.wall_offset_l) ||
      (load_blob.wall_offset_fr != save_blob.wall_offset_fr) ||
      (load_blob.wall_offset_fl != save_blob.wall_offset_fl) ||
      (load_blob.imu_offset_z != save_blob.imu_offset_z))
  {
    trace_printf("[NVM-TEST][Sensor] compare: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Sensor] save/load compare: PASS\r\n");
  return true;
}

bool f413_diag_run_maze_nvm_test(void)
{
  uint16_t save_cells[F413_DIAG_TEST_MAZE_CELLS];
  uint16_t load_cells[F413_DIAG_TEST_MAZE_CELLS];
  HAL_StatusTypeDef st;
  uint32_t i;

  f413_diag_fill_expected_maze_cells(save_cells, F413_DIAG_TEST_MAZE_CELLS);

  st = nvm_maze_save_map(save_cells, F413_DIAG_TEST_MAZE_CELLS);
  if (st != HAL_OK)
  {
    trace_printf("[NVM-TEST][Maze] save: FAIL (HAL=%d)\r\n", (int)st);
    return false;
  }

  memset(load_cells, 0, sizeof(load_cells));
  if (!nvm_maze_load_map(load_cells, F413_DIAG_TEST_MAZE_CELLS))
  {
    trace_printf("[NVM-TEST][Maze] load: FAIL\r\n");
    return false;
  }

  for (i = 0U; i < F413_DIAG_TEST_MAZE_CELLS; i++)
  {
    if (load_cells[i] != save_cells[i])
    {
      trace_printf("[NVM-TEST][Maze] compare: FAIL idx=%lu saved=0x%04X loaded=0x%04X\r\n",
                   (unsigned long)i,
                   (unsigned int)save_cells[i],
                   (unsigned int)load_cells[i]);
      return false;
    }
  }

  trace_printf("[NVM-TEST][Maze] save/load compare: PASS\r\n");
  return true;
}

bool f413_diag_verify_distance_nvm_load_only(void)
{
  if (!nvm_params_distance_load_and_apply())
  {
    trace_printf("[NVM-TEST][Distance] load_only: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Distance] load_only: PASS\r\n");
  return true;
}

bool f413_diag_verify_sensor_nvm_load_only(void)
{
  nvm_sensor_params_t expected;
  nvm_sensor_params_t load_blob;

  f413_diag_fill_expected_sensor_params(&expected);
  memset(&load_blob, 0, sizeof(load_blob));

  if (!nvm_params_sensor_load(&load_blob))
  {
    trace_printf("[NVM-TEST][Sensor] load_only: FAIL(load)\r\n");
    return false;
  }

  if ((load_blob.base_l != expected.base_l) ||
      (load_blob.base_r != expected.base_r) ||
      (load_blob.base_f != expected.base_f) ||
      (load_blob.wall_offset_r != expected.wall_offset_r) ||
      (load_blob.wall_offset_l != expected.wall_offset_l) ||
      (load_blob.wall_offset_fr != expected.wall_offset_fr) ||
      (load_blob.wall_offset_fl != expected.wall_offset_fl) ||
      (load_blob.imu_offset_z != expected.imu_offset_z))
  {
    trace_printf("[NVM-TEST][Sensor] load_only: FAIL(compare)\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Sensor] load_only: PASS\r\n");
  return true;
}

bool f413_diag_verify_maze_nvm_load_only(void)
{
  uint16_t expected[F413_DIAG_TEST_MAZE_CELLS];
  uint16_t loaded[F413_DIAG_TEST_MAZE_CELLS];
  uint32_t i;

  f413_diag_fill_expected_maze_cells(expected, F413_DIAG_TEST_MAZE_CELLS);
  memset(loaded, 0, sizeof(loaded));

  if (!nvm_maze_load_map(loaded, F413_DIAG_TEST_MAZE_CELLS))
  {
    trace_printf("[NVM-TEST][Maze] load_only: FAIL(load)\r\n");
    return false;
  }

  for (i = 0U; i < F413_DIAG_TEST_MAZE_CELLS; i++)
  {
    if (loaded[i] != expected[i])
    {
      trace_printf("[NVM-TEST][Maze] load_only: FAIL(compare idx=%lu)\r\n", (unsigned long)i);
      return false;
    }
  }

  trace_printf("[NVM-TEST][Maze] load_only: PASS\r\n");
  return true;
}

bool f413_diag_run_trace_log_nvm_test(void)
{
  uint8_t expected[F413_DIAG_TEST_TRACE_BYTES];
  uint8_t loaded[F413_DIAG_TEST_TRACE_BYTES];
  nvm_status_t st;

  f413_diag_fill_expected_trace_bytes(expected, F413_DIAG_TEST_TRACE_BYTES);
  memset(loaded, 0, sizeof(loaded));

  st = nvm_write(NVM_AREA_TRACE_LOG,
                 F413_DIAG_TEST_TRACE_OFFSET,
                 expected,
                 sizeof(expected));
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[NVM-TEST][Trace] save: FAIL (NVM=%d)\r\n", (int)st);
    return false;
  }

  st = nvm_read(NVM_AREA_TRACE_LOG,
                F413_DIAG_TEST_TRACE_OFFSET,
                loaded,
                sizeof(loaded));
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[NVM-TEST][Trace] load: FAIL (NVM=%d)\r\n", (int)st);
    return false;
  }

  if (memcmp(expected, loaded, sizeof(expected)) != 0)
  {
    trace_printf("[NVM-TEST][Trace] compare: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Trace] save/load compare: PASS\r\n");
  return true;
}

bool f413_diag_verify_trace_log_nvm_load_only(void)
{
  uint8_t expected[F413_DIAG_TEST_TRACE_BYTES];
  uint8_t loaded[F413_DIAG_TEST_TRACE_BYTES];
  nvm_status_t st;

  f413_diag_fill_expected_trace_bytes(expected, F413_DIAG_TEST_TRACE_BYTES);
  memset(loaded, 0, sizeof(loaded));

  st = nvm_read(NVM_AREA_TRACE_LOG,
                F413_DIAG_TEST_TRACE_OFFSET,
                loaded,
                sizeof(loaded));
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[NVM-TEST][Trace] load_only: FAIL(load NVM=%d)\r\n", (int)st);
    return false;
  }

  if (memcmp(expected, loaded, sizeof(expected)) != 0)
  {
    trace_printf("[NVM-TEST][Trace] load_only: FAIL(compare)\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Trace] load_only: PASS\r\n");
  return true;
}

void f413_diag_run_all_nvm_tests(void)
{
  bool distance_ok = f413_diag_run_distance_nvm_test();
  bool sensor_ok = f413_diag_run_sensor_nvm_test();
  bool maze_ok = f413_diag_run_maze_nvm_test();
  bool trace_ok = f413_diag_run_trace_log_nvm_test();

  trace_printf("[NVM-TEST][Overall] %s\r\n", (distance_ok && sensor_ok && maze_ok && trace_ok) ? "PASS" : "FAIL");
}

void f413_diag_verify_all_nvm_load_only(void)
{
  bool distance_ok = f413_diag_verify_distance_nvm_load_only();
  bool sensor_ok = f413_diag_verify_sensor_nvm_load_only();
  bool maze_ok = f413_diag_verify_maze_nvm_load_only();
  bool trace_ok = f413_diag_verify_trace_log_nvm_load_only();

  trace_printf("[NVM-TEST][Overall][LoadOnly] %s\r\n", (distance_ok && sensor_ok && maze_ok && trace_ok) ? "PASS" : "FAIL");
}
