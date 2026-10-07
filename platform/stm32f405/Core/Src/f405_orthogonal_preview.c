#include "f405_orthogonal_preview.h"
#include "params.h"
#include "shortest_run_params.h"

#include <string.h>

_Static_assert(MAZE_SIZE == 16, "classic preview requires a 16x16 profile");
static NfCompactWorkspace workspace;
static NfRouteMaze maze;

bool f405_orthogonal_config(uint8_t mode, uint8_t case_index,
                            NfOrthogonalPlannerConfig *config)
{
    if (!config || mode<2 || mode>7 || case_index<1 || case_index>9) return false;
    const ShortestRunModeParams_t *modes[]={&shortestRunModeParams2,&shortestRunModeParams3,
        &shortestRunModeParams4,&shortestRunModeParams5,&shortestRunModeParams6,&shortestRunModeParams7};
    const ShortestRunCaseParams_t *cases[]={shortestRunCaseParamsMode2,shortestRunCaseParamsMode3,
        shortestRunCaseParamsMode4,shortestRunCaseParamsMode5,shortestRunCaseParamsMode6,shortestRunCaseParamsMode7};
    const ShortestRunModeParams_t *m=modes[mode-2];
    const ShortestRunCaseParams_t *p=&cases[mode-2][case_index-1];
    *config=(NfOrthogonalPlannerConfig){
        .half_cell_mm=DIST_HALF_SEC,.start_offset_mm=DIST_FIRST_SEC,
        .straight={.vmax_mm_s=p->velocity_straight,.switch_velocity_mm_s=m->accel_switch_velocity,
            .accel_low_mm_s2=p->acceleration_straight,.accel_high_mm_s2=p->acceleration_straight_dash},
        .turn_environment={.omega_cap_deg_s=0,.rounding_scale=TURN_OMEGA_PROFILE_ROUNDING_SCALE},
        .small_90={true,m->velocity_turn90,m->alpha_turn90,
            case_index<=2?m->angle_turn_90:90,m->dist_offset_in,m->dist_offset_out},
        .large_90={true,m->velocity_l_turn_90,m->alpha_l_turn_90,
            case_index<=2?m->angle_l_turn_90:90,m->dist_l_turn_in_90,m->dist_l_turn_out_90},
        .large_180={true,m->velocity_l_turn_180,m->alpha_l_turn_180,
            case_index<=2?m->angle_l_turn_180:180,m->dist_l_turn_in_180,m->dist_l_turn_out_180},
        .allow_large_turns=(case_index==1?m->makepath_type_case3:m->makepath_type_case47)>0};
    return true;
}

static bool overlaps(const void *a,size_t na,const void *b,size_t nb)
{
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return x<=y ? y-x<na : x-y<nb;
}

NfRoutePlanStatus f405_orthogonal_preview(uint8_t mode,uint8_t case_index,
    const uint8_t *map_cells,size_t cell_count,
    uint16_t *output,size_t capacity,NfCompactResult *result)
{
    NfOrthogonalPlannerConfig config;
    if (!map_cells || !output || !result || capacity==0 || capacity>NF_COMPACT_PATH_CAPACITY ||
        cell_count!=MAZE_SIZE*MAZE_SIZE ||
        overlaps(map_cells,cell_count,output,capacity*sizeof(*output)) ||
        overlaps(map_cells,cell_count,result,sizeof(*result)) ||
        !f405_orthogonal_config(mode,case_index,&config)) return NF_ROUTE_PLAN_INVALID_ARGUMENT;
    memset(&maze,0,sizeof(maze)); maze.width=MAZE_SIZE; maze.height=MAZE_SIZE;
    for (unsigned y=0;y<MAZE_SIZE;++y) for (unsigned x=0;x<MAZE_SIZE;++x) {
        uint8_t value=map_cells[y*MAZE_SIZE+x];
        maze.walls[y][x]=(uint8_t)((value>>4)|(value&15U));
    }
    for (unsigned y=0;y<MAZE_SIZE;++y) for (unsigned x=0;x<MAZE_SIZE;++x) {
        if (x==0) maze.walls[y][x]|=1;
        if (y==0) maze.walls[y][x]|=2;
        if (x+1==MAZE_SIZE) maze.walls[y][x]|=4;
        if (y+1==MAZE_SIZE) maze.walls[y][x]|=8;
        if (x+1<MAZE_SIZE && ((maze.walls[y][x]&4)||(maze.walls[y][x+1]&1))) {
            maze.walls[y][x]|=4; maze.walls[y][x+1]|=1;
        }
        if (y+1<MAZE_SIZE && ((maze.walls[y][x]&8)||(maze.walls[y+1][x]&2))) {
            maze.walls[y][x]|=8; maze.walls[y+1][x]|=2;
        }
    }
    static const int goals[9][2]={{GOAL1_X,GOAL1_Y},{GOAL2_X,GOAL2_Y},{GOAL3_X,GOAL3_Y},
        {GOAL4_X,GOAL4_Y},{GOAL5_X,GOAL5_Y},{GOAL6_X,GOAL6_Y},
        {GOAL7_X,GOAL7_Y},{GOAL8_X,GOAL8_Y},{GOAL9_X,GOAL9_Y}};
    for (size_t i=0;i<9;++i) {
        int x=goals[i][0],y=goals[i][1];
        if (x==0 && y==0) continue;
        if (x<0 || y<0 || x>=MAZE_SIZE || y>=MAZE_SIZE) return NF_ROUTE_PLAN_INVALID_CONFIG;
        maze.goals[y][x]=true;
    }
    NfOrthogonalPlannerRequest request={START_X,START_Y,NF_ROUTE_DIR_NORTH};
    return nf_compact_orthogonal_plan(&maze,&config,&request,&workspace,output,capacity,result);
}
