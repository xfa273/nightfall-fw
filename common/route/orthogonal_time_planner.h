#ifndef NIGHTFALL_COMMON_ROUTE_ORTHOGONAL_TIME_PLANNER_H
#define NIGHTFALL_COMMON_ROUTE_ORTHOGONAL_TIME_PLANNER_H

#include "motion_time.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NF_ROUTE_MAZE_MAX_SIZE 32U
#define NF_ROUTE_MAX_ACTIONS 4096U

#define NF_ROUTE_WALL_NORTH 0x08U
#define NF_ROUTE_WALL_EAST  0x04U
#define NF_ROUTE_WALL_SOUTH 0x02U
#define NF_ROUTE_WALL_WEST  0x01U

typedef enum {
    NF_ROUTE_DIR_NORTH = 0,
    NF_ROUTE_DIR_EAST = 1,
    NF_ROUTE_DIR_SOUTH = 2,
    NF_ROUTE_DIR_WEST = 3,
} NfRouteDirection;

typedef enum {
    NF_ROUTE_SIDE_NONE = 0,
    NF_ROUTE_SIDE_RIGHT,
    NF_ROUTE_SIDE_LEFT,
} NfRouteSide;

typedef enum {
    NF_ROUTE_MOTION_START_OFFSET = 0,
    NF_ROUTE_MOTION_STRAIGHT,
    NF_ROUTE_MOTION_SMALL_90,
    NF_ROUTE_MOTION_LARGE_90,
    NF_ROUTE_MOTION_LARGE_180,
} NfRouteMotionKind;

typedef struct {
    uint8_t width;
    uint8_t height;
    uint8_t walls[NF_ROUTE_MAZE_MAX_SIZE][NF_ROUTE_MAZE_MAX_SIZE];
    bool goals[NF_ROUTE_MAZE_MAX_SIZE][NF_ROUTE_MAZE_MAX_SIZE];
} NfRouteMaze;

typedef struct {
    double half_cell_mm;
    double start_offset_mm;
    NfLinearLimits straight;
    NfTurnEnvironment turn_environment;
    NfTurnSpec small_90;
    NfTurnSpec large_90;
    NfTurnSpec large_180;
    bool allow_large_turns;
} NfOrthogonalPlannerConfig;

typedef struct {
    uint8_t start_x;
    uint8_t start_y;
    NfRouteDirection start_heading;
} NfOrthogonalPlannerRequest;

typedef struct {
    NfRouteMotionKind kind;
    NfRouteSide side;
    uint8_t start_x;
    uint8_t start_y;
    uint8_t end_x;
    uint8_t end_y;
    NfRouteDirection start_heading;
    NfRouteDirection end_heading;
    uint16_t logical_cells;
    double distance_mm;
    double entry_velocity_mm_s;
    double exit_velocity_mm_s;
    uint64_t duration_us;
    bool has_goal_cross;
    uint16_t goal_cross_after_cells;
    uint8_t goal_x;
    uint8_t goal_y;
    uint64_t goal_cross_time_us;
} NfRouteMotion;

typedef struct {
    NfRouteMotion actions[NF_ROUTE_MAX_ACTIONS];
    size_t action_count;
    uint8_t goal_x;
    uint8_t goal_y;
    /* Logical heading after the crossing action, not instantaneous turn yaw. */
    NfRouteDirection goal_heading;
    uint16_t post_goal_extension_cells;
    uint64_t goal_entry_us;
    uint64_t stop_us;
    double goal_entry_velocity_mm_s;
    uint32_t expanded_states;
    uint32_t relaxed_edges;
} NfOrthogonalRoutePlan;

typedef enum {
    NF_ROUTE_PLAN_OK = 0,
    NF_ROUTE_PLAN_INVALID_ARGUMENT,
    NF_ROUTE_PLAN_INVALID_MAZE,
    NF_ROUTE_PLAN_INVALID_CONFIG,
    NF_ROUTE_PLAN_NO_PATH,
    NF_ROUTE_PLAN_CAPACITY,
    NF_ROUTE_PLAN_OVERFLOW,
} NfRoutePlanStatus;

typedef struct {
    bool valid;
    size_t action_index;
    char message[160];
} NfRouteValidation;

const char *nf_route_plan_status_name(NfRoutePlanStatus status);
const char *nf_route_direction_name(NfRouteDirection direction);
const char *nf_route_motion_name(NfRouteMotionKind kind, NfRouteSide side);

bool nf_route_maze_init(NfRouteMaze *maze, uint8_t width, uint8_t height);
bool nf_route_maze_set_wall(NfRouteMaze *maze, uint8_t x, uint8_t y,
                            NfRouteDirection direction);
bool nf_route_maze_add_boundaries(NfRouteMaze *maze);
bool nf_route_maze_can_move(const NfRouteMaze *maze, uint8_t x, uint8_t y,
                            NfRouteDirection direction);

NfRoutePlanStatus nf_orthogonal_time_plan(const NfRouteMaze *maze,
                                          const NfOrthogonalPlannerConfig *config,
                                          const NfOrthogonalPlannerRequest *request,
                                          NfOrthogonalRoutePlan *out);

bool nf_orthogonal_route_validate(const NfRouteMaze *maze,
                                  const NfOrthogonalPlannerConfig *config,
                                  const NfOrthogonalPlannerRequest *request,
                                  const NfOrthogonalRoutePlan *plan,
                                  NfRouteValidation *validation);

#ifdef __cplusplus
}
#endif

#endif
