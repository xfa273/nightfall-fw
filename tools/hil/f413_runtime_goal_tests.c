/* Exercise real goal consumers with two deliberately different machine profiles.
 * Hardware/NVM writes are not linked; ASan/UBSan cover maze indexing. */
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_search_step.c"
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_route_preview.c"
#include "solver.h"
#include "maze_grid.h"
#include "sensor_distance.h"

uint16_t map[MAZE_SIZE][MAZE_SIZE], smap[MAZE_SIZE][MAZE_SIZE];
bool visited[MAZE_SIZE][MAZE_SIZE];
uint16_t path[ROUTE_MAX_LEN];
volatile struct coordinate_and_direction mouse;
volatile mouse_flags MF;

nvm_status_t nvm_read(nvm_area_t a, uint32_t o, void *p, size_t n)
{ (void)a; (void)o; (void)p; (void)n; abort(); }
nvm_status_t nvm_write(nvm_area_t a, uint32_t o, const void *p, size_t n)
{ (void)a; (void)o; (void)p; (void)n; abort(); }
nvm_status_t nvm_erase(nvm_area_t a) { (void)a; abort(); }
bool nvm_params_distance_load_and_apply(void) { return false; }
bool nvm_maze_load_map(uint16_t *cells, uint32_t count)
{ assert(count == MAZE_SIZE * MAZE_SIZE); memcpy(cells, map, sizeof(map)); return true; }
void load_map_from_eeprom(void) {} /* The test supplied an all-known open maze. */

int main(int argc, char **argv)
{
  assert(argc == 2);
  const unsigned rev = (unsigned)atoi(argv[1]);
  const uint32_t uid[] = {0x00280047U, 0x31335117U, 0x34313932U};
  nvm_identity_block_t id = {0};
  id.magic = NVM_IDENTITY_MAGIC;
  id.schema_version = NVM_IDENTITY_SCHEMA_VERSION;
  id.length = sizeof(id);
  id.family = NVM_FAMILY_MINI;
  id.board_id = rev << 16;
  id.hw_rev_major = rev;
  id.unit_serial = 1;
  memcpy(id.mcu_uid, uid, sizeof(uid));
  for (size_t i = 16; i < sizeof(id); ++i) id.crc += ((const uint8_t *)&id)[i];
  assert(f413_machine_boot(NVM_STATUS_OK, &id, uid) == F413_MACHINE_OK);
  const f413_param_profile_t *p = rev == 2 ? &f413_profile_mini_r2 : &f413_profile_mini_r3;
#define X(type, name) assert(name == p->scalar->v_##name);
#include "f413_param_fields.def"
#undef X
  assert(GOAL_X == (rev == 2 ? 1 : 0) && GOAL_Y == (rev == 2 ? 0 : 8));
  assert(START_X == (rev == 2 ? 2 : 0) && START_Y == (rev == 2 ? 3 : 0));
  assert(memcmp(searchRunParams, p->search, sizeof(f413_machine_params()->search)) == 0);
#define CHECK_MODE(n) do { \
  assert(memcmp(&shortestRunModeParams##n, p->modes[n-2], sizeof(*p->modes[n-2])) == 0); \
  assert(memcmp(shortestRunCaseParamsMode##n, p->cases[n-2], sizeof(f413_machine_params()->cases[0])) == 0); \
} while (0)
  CHECK_MODE(2); CHECK_MODE(3); CHECK_MODE(4); CHECK_MODE(5); CHECK_MODE(6); CHECK_MODE(7);
#undef CHECK_MODE
  for (unsigned y = 0; y < MAZE_SIZE; ++y) {
    for (unsigned x = 0; x < MAZE_SIZE; ++x) {
      const uint16_t walls = (x == 0 ? 1 : 0) | (y == 0 ? 2 : 0) |
          (x == MAZE_SIZE-1 ? 4 : 0) | (y == MAZE_SIZE-1 ? 8 : 0);
      map[y][x] = walls | (walls << 4);
    }
  }
  assert(f413_search_step_is_goal_cell(GOAL_X, GOAL_Y));
  assert(!f413_search_step_is_goal_cell(rev == 2 ? 0 : 1, rev == 2 ? 8 : 0));
  assert(f413_search_step_is_goal_cell(15, 15)); /* Fixture uses GOAL9 too. */
  assert(!f413_search_step_is_goal_cell(0, 0));
  assert(f413_search_step_make_smap(START_X, START_Y, F413_SEARCH_STEP_TARGET_GOAL) ==
      abs(GOAL_X - START_X) + abs(GOAL_Y - START_Y));
  assert(smap[GOAL_Y][GOAL_X] == 0 && smap[15][15] == 0);
  assert(smap[START_Y][START_X] == abs(GOAL_X - START_X) + abs(GOAL_Y - START_Y));
  assert(f413_search_step_make_smap(GOAL_X, GOAL_Y, F413_SEARCH_STEP_TARGET_START) ==
      abs(GOAL_X - START_X) + abs(GOAL_Y - START_Y));
  assert(smap[START_Y][START_X] == 0);
  f413_rp_maze_t run_maze;
  f413_rp_maze_source_t source;
  assert(f413_rp_load_run_maze(&run_maze, &source));
  assert(run_maze.goal_count == 2);
  assert(f413_rp_goal_at(&run_maze, GOAL_X, GOAL_Y));
  assert(f413_rp_goal_at(&run_maze, 15, 15));
  assert(!f413_rp_goal_at(&run_maze, rev == 2 ? 0 : 1, rev == 2 ? 8 : 0));
  assert(solver_build_path(2, 1));
  assert(path_cell[START_Y][START_X] && path_cell[GOAL_Y][GOAL_X]);
  if (rev == 3) {
    for (unsigned y = 0; y <= 8; ++y) assert(path_cell[y][0]);
    for (unsigned x = 1; x < MAZE_SIZE; ++x)
      for (unsigned y = 0; y < MAZE_SIZE; ++y) assert(!path_cell[y][x]);
  }
  sensor_distance_init();
  const uint16_t mm[] = {10, 20}, ad[] = {200, 100};
  assert(sensor_distance_set_lut_l(mm, ad, 2) == 0);
  assert(SENSOR_DIST_GAIN == (rev == 2 ? 1.25f : 2.0f));
  assert(sensor_distance_from_l(200) == 10.0f * SENSOR_DIST_GAIN);
  printf("PASS: mini_r%u runtime fields, start/9 goals, search BFS, shortest path, sensor gain\n", rev);
  return 0;
}
