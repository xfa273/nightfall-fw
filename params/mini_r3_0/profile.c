/* Compiled into the common F413 binary; selected once from NVM identity. */
#define F413_PARAMS_DEFINITION
#include "f413_machine.h"
#include "params.h"
#define searchRunParams mini_r3_search
#define shortestRunModeParams2 mini_r3_mode2
#define shortestRunCaseParamsMode2 mini_r3_cases2
#define shortestRunModeParams3 mini_r3_mode3
#define shortestRunCaseParamsMode3 mini_r3_cases3
#define shortestRunModeParams4 mini_r3_mode4
#define shortestRunCaseParamsMode4 mini_r3_cases4
#define shortestRunModeParams5 mini_r3_mode5
#define shortestRunCaseParamsMode5 mini_r3_cases5
#define shortestRunModeParams6 mini_r3_mode6
#define shortestRunCaseParamsMode6 mini_r3_cases6
#define shortestRunModeParams7 mini_r3_mode7
#define shortestRunCaseParamsMode7 mini_r3_cases7
#include "search_run_params_split.c"
#include "shortest_run_params_split.c"
#define sensor_distance_load_profile_luts mini_r3_load_sensor_luts
#include "sensor_distance_lut.c"

/* Resolver copies fixed counts; reject truncated tables at compile time. */
_Static_assert(sizeof(mini_r3_search) / sizeof(mini_r3_search[0]) == 2, "search profile count");
_Static_assert(sizeof(mini_r3_cases2) / sizeof(mini_r3_cases2[0]) == 9, "mode 2 case count");
_Static_assert(sizeof(mini_r3_cases3) / sizeof(mini_r3_cases3[0]) == 9, "mode 3 case count");
_Static_assert(sizeof(mini_r3_cases4) / sizeof(mini_r3_cases4[0]) == 9, "mode 4 case count");
_Static_assert(sizeof(mini_r3_cases5) / sizeof(mini_r3_cases5[0]) == 9, "mode 5 case count");
_Static_assert(sizeof(mini_r3_cases6) / sizeof(mini_r3_cases6[0]) == 9, "mode 6 case count");
_Static_assert(sizeof(mini_r3_cases7) / sizeof(mini_r3_cases7[0]) == 9, "mode 7 case count");
static const f413_scalar_params_t scalar = {
#define X(type, key) .v_##key = key,
#include "f413_param_fields.def"
#undef X
};
const f413_param_profile_t f413_profile_mini_r3 = {
  .id = 0x00030001U, .family = NVM_FAMILY_MINI, .board_id = 0x00030000U,
  .name = PARAMS_TUNE_VERSION, .scalar = &scalar, .search = mini_r3_search,
  .modes = {&mini_r3_mode2, &mini_r3_mode3, &mini_r3_mode4, &mini_r3_mode5, &mini_r3_mode6, &mini_r3_mode7},
  .cases = {mini_r3_cases2, mini_r3_cases3, mini_r3_cases4, mini_r3_cases5, mini_r3_cases6, mini_r3_cases7},
  .route_precomputed_compatible = false,
  .load_sensor_luts = mini_r3_load_sensor_luts
};
