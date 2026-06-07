#include "f413_nvm_diag.h"

#include <string.h>

#include "f413_diag.h"
#include "nvm_params.h"
#include "nvm_trace_log.h"
#include "params.h"
#include "stm32f4xx_hal.h"
#include "trace.h"

#define F413_NVM_DIAG_MAZE_WALL_KNOWN_MASK (0x0FU)
#define F413_NVM_DIAG_SEARCH_MAP_CELL_COUNT ((uint32_t)(MAZE_SIZE * MAZE_SIZE))

const char* f413_nvm_diag_identity_family_name(uint32_t family)
{
  switch (family)
  {
    case NVM_FAMILY_MINI:
      return "mini";
    case NVM_FAMILY_CLASSIC:
      return "classic";
    default:
      return "unknown";
  }
}

void f413_nvm_diag_emit_identity_meta(const nvm_identity_block_t* id)
{
  const char* family_name;

  if (id == NULL)
  {
    return;
  }

  family_name = f413_nvm_diag_identity_family_name(id->family);
  trace_printf("#fw_family=%lu\r\n", (unsigned long)id->family);
  trace_printf("#fw_family_name=%s\r\n", family_name);
  trace_printf("#fw_machine_name=%s_r%u_%u\r\n",
               family_name,
               (unsigned int)id->hw_rev_major,
               (unsigned int)id->hw_rev_minor);
  trace_printf("#fw_machine_unit=%s_r%u_%u_unit%03lu\r\n",
               family_name,
               (unsigned int)id->hw_rev_major,
               (unsigned int)id->hw_rev_minor,
               (unsigned long)id->unit_serial);
  trace_printf("#fw_board_id=%lu\r\n", (unsigned long)id->board_id);
  trace_printf("#fw_board_id_hex=0x%08lX\r\n", (unsigned long)id->board_id);
  trace_printf("#fw_hw_rev=%u.%u\r\n",
               (unsigned int)id->hw_rev_major,
               (unsigned int)id->hw_rev_minor);
  trace_printf("#fw_unit_serial=%lu\r\n", (unsigned long)id->unit_serial);
}

void f413_nvm_diag_run_identity_status_once(nvm_status_t status,
                                            const nvm_identity_block_t* identity)
{
  if ((status == NVM_STATUS_OK) && (identity != NULL))
  {
    const char* family_name = f413_nvm_diag_identity_family_name(identity->family);
    trace_printf("[IDENTITY] status=OK family=%s(%lu) board=0x%08lX rev=%u.%u unit=%lu cap=0x%08lX\r\n",
                 family_name,
                 (unsigned long)identity->family,
                 (unsigned long)identity->board_id,
                 (unsigned int)identity->hw_rev_major,
                 (unsigned int)identity->hw_rev_minor,
                 (unsigned long)identity->unit_serial,
                 (unsigned long)identity->capability_flags);
    trace_printf("[IDENTITY] uid=%08lX-%08lX-%08lX\r\n",
                 (unsigned long)HAL_GetUIDw0(),
                 (unsigned long)HAL_GetUIDw1(),
                 (unsigned long)HAL_GetUIDw2());
  }
  else
  {
    trace_printf("[IDENTITY] status=%d uid=%08lX-%08lX-%08lX\r\n",
                 (int)status,
                 (unsigned long)HAL_GetUIDw0(),
                 (unsigned long)HAL_GetUIDw1(),
                 (unsigned long)HAL_GetUIDw2());
  }
}

void f413_nvm_diag_run_sensor_params_status_once(void)
{
  nvm_sensor_params_t params;
  bool loaded;

  memset(&params, 0, sizeof(params));
  loaded = nvm_params_sensor_load(&params);
  if (!loaded)
  {
    nvm_params_sensor_defaults(&params);
  }

  trace_printf("[SENSOR-PARAM] source=%s base_l=%u base_r=%u base_f=%u off_r=%u off_l=%u off_fr=%u off_fl=%u imu_z=%.3f\r\n",
               loaded ? "NVM" : "default",
               (unsigned int)params.base_l,
               (unsigned int)params.base_r,
               (unsigned int)params.base_f,
               (unsigned int)params.wall_offset_r,
               (unsigned int)params.wall_offset_l,
               (unsigned int)params.wall_offset_fr,
               (unsigned int)params.wall_offset_fl,
               (double)params.imu_offset_z);
  trace_printf("[SENSOR-PARAM] save is intentionally not performed by OP mode9 case9\r\n");
}

void f413_nvm_diag_run_nvm_status_once(void)
{
  static uint16_t cells[F413_NVM_DIAG_SEARCH_MAP_CELL_COUNT];
  nvm_trace_log_header_t header;
  uint32_t known_count = 0U;
  uint32_t i;
  bool distance_ok;
  bool sensor_ok;
  bool maze_ok;
  nvm_status_t trace_st;
  nvm_sensor_params_t sensor_params;

  distance_ok = nvm_params_distance_load_and_apply();
  sensor_ok = nvm_params_sensor_load(&sensor_params);
  maze_ok = nvm_maze_load_map(cells, F413_NVM_DIAG_SEARCH_MAP_CELL_COUNT);
  trace_st = nvm_trace_log_get_header(&header);

  if (maze_ok)
  {
    for (i = 0U; i < F413_NVM_DIAG_SEARCH_MAP_CELL_COUNT; i++)
    {
      if ((cells[i] & F413_NVM_DIAG_MAZE_WALL_KNOWN_MASK) != 0U)
      {
        known_count++;
      }
    }
  }

  trace_printf("[NVM-STATUS] distance=%s sensor=%s maze=%s maze_known=%lu trace=%s(%d)\r\n",
               distance_ok ? "OK" : "MISS",
               sensor_ok ? "OK" : "MISS",
               maze_ok ? "OK" : "MISS",
               (unsigned long)known_count,
               (trace_st == NVM_STATUS_OK) ? "OK" : "MISS",
               (int)trace_st);
  if (trace_st == NVM_STATUS_OK)
  {
    trace_printf("[NVM-STATUS] trace ver=0x%08lX rec_size=%lu cap=%lu write=%lu total=%lu\r\n",
                 (unsigned long)header.version,
                 (unsigned long)header.record_size,
                 (unsigned long)header.record_capacity,
                 (unsigned long)header.write_index,
                 (unsigned long)header.total_records);
  }
}

bool f413_nvm_diag_run_distance_test(void)
{
  return f413_diag_run_distance_nvm_test();
}

bool f413_nvm_diag_run_sensor_test(void)
{
  return f413_diag_run_sensor_nvm_test();
}

bool f413_nvm_diag_run_maze_test(void)
{
  return f413_diag_run_maze_nvm_test();
}

bool f413_nvm_diag_run_trace_log_test(void)
{
  return f413_diag_run_trace_log_nvm_test();
}

bool f413_nvm_diag_verify_distance_load_only(void)
{
  return f413_diag_verify_distance_nvm_load_only();
}

bool f413_nvm_diag_verify_sensor_load_only(void)
{
  return f413_diag_verify_sensor_nvm_load_only();
}

bool f413_nvm_diag_verify_maze_load_only(void)
{
  return f413_diag_verify_maze_nvm_load_only();
}

bool f413_nvm_diag_verify_trace_log_load_only(void)
{
  return f413_diag_verify_trace_log_nvm_load_only();
}

void f413_nvm_diag_run_all_tests(void)
{
  f413_diag_run_all_nvm_tests();
}

void f413_nvm_diag_verify_all_load_only(void)
{
  f413_diag_verify_all_nvm_load_only();
}
