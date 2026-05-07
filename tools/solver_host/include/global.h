#ifndef NIGHTFALL_SOLVER_HOST_GLOBAL_H_
#define NIGHTFALL_SOLVER_HOST_GLOBAL_H_

#include <params.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef ROUTE_MAX_LEN
#define ROUTE_MAX_LEN 1024
#endif

#if ((MAZE_SIZE * MAZE_SIZE) > ROUTE_MAX_LEN)
#error "ROUTE_MAX_LEN is too small for current MAZE_SIZE"
#endif

typedef union {
    uint64_t FLAGS;
    struct {
        uint64_t SCND : 1;
        uint64_t RETURN : 1;
        uint64_t CTRL : 1;
        uint64_t OVERRIDE : 1;
        uint64_t GET_LOG_1 : 1;
        uint64_t GET_LOG_2 : 1;
        uint64_t SPEED_x2 : 1;
        uint64_t SPEED_x3 : 1;
        uint64_t SPARE_16 : 1;
        uint64_t TURN_1ST : 1;
        uint64_t SLALOM_R : 1;
        uint64_t SLALOM_L : 1;
        uint64_t F_WALL : 1;
        uint64_t F_WALL_STOP : 1;
        uint64_t GOALED : 1;
        uint64_t SUCTION : 1;
        uint64_t R_WALL : 1;
        uint64_t L_WALL : 1;
        uint64_t R_WALL_END : 1;
        uint64_t L_WALL_END : 1;
        uint64_t WALL_END : 1;
        uint64_t CTRL_DIAGONAL : 1;
        uint64_t WALL_ALIGN : 1;
        uint64_t SEARCH_HALF_RATE : 1;
        uint64_t SPARE_09 : 1;
        uint64_t SPARE_10 : 1;
        uint64_t SPARE_11 : 1;
        uint64_t SPARE_12 : 1;
        uint64_t SPARE_13 : 1;
        uint64_t SPARE_14 : 1;
        uint64_t RUNNING : 1;
        uint64_t FAILED : 1;
    } FLAG;
} mouse_flags;

typedef struct {
    bool test_mode_run;
    bool disable_wall_end_correction;
    bool disable_front_wall_correction;
    bool sensor_log_enabled;
    bool angle_accum_mode;
} DebugFlags_t;

#ifdef MAIN_C_
volatile mouse_flags MF;
volatile float g_ctrl_dt;
volatile DebugFlags_t g_debug;
uint16_t map[MAZE_SIZE][MAZE_SIZE];
uint16_t smap[MAZE_SIZE][MAZE_SIZE];
bool visited[MAZE_SIZE][MAZE_SIZE];
uint16_t closest_unvisited_x;
uint16_t closest_unvisited_y;
bool search_end;
uint16_t wall_info;
uint16_t goal_x;
uint16_t goal_y;
uint16_t route[ROUTE_MAX_LEN];
uint16_t path[ROUTE_MAX_LEN];
uint16_t r_cnt;
float strait_count;
float accel;
float acceled_count;
bool known_straight;
bool acceled;
float sensor_kx;
float fwall_kx;
bool g_goal_is_start;
bool g_second_phase_search;
float g_search_coast_mm;
#else
extern volatile mouse_flags MF;
extern volatile float g_ctrl_dt;
extern volatile DebugFlags_t g_debug;
extern uint16_t map[MAZE_SIZE][MAZE_SIZE];
extern uint16_t smap[MAZE_SIZE][MAZE_SIZE];
extern bool visited[MAZE_SIZE][MAZE_SIZE];
extern uint16_t closest_unvisited_x;
extern uint16_t closest_unvisited_y;
extern bool search_end;
extern uint16_t wall_info;
extern uint16_t goal_x;
extern uint16_t goal_y;
extern uint16_t route[ROUTE_MAX_LEN];
extern uint16_t path[ROUTE_MAX_LEN];
extern uint16_t r_cnt;
extern float strait_count;
extern float accel;
extern float acceled_count;
extern bool known_straight;
extern bool acceled;
extern float sensor_kx;
extern float fwall_kx;
extern bool g_goal_is_start;
extern bool g_second_phase_search;
extern float g_search_coast_mm;
#endif

#define g_test_mode_run (g_debug.test_mode_run)
#define g_disable_wall_end_correction (g_debug.disable_wall_end_correction)
#define g_disable_front_wall_correction (g_debug.disable_front_wall_correction)
#define g_sensor_log_enabled (g_debug.sensor_log_enabled)
#define g_angle_accum_mode (g_debug.angle_accum_mode)

void load_map_from_eeprom(void);

#include "path.h"

#endif
