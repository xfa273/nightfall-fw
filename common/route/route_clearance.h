#ifndef NIGHTFALL_COMMON_ROUTE_ROUTE_CLEARANCE_H
#define NIGHTFALL_COMMON_ROUTE_ROUTE_CLEARANCE_H

#include "motion_time.h"
#include "orthogonal_time_planner.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NF_CLEARANCE_OK = 0,
    NF_CLEARANCE_INVALID_ARGUMENT,
    NF_CLEARANCE_COLLISION,
    NF_CLEARANCE_OVERFLOW,
} NfClearanceStatus;

typedef struct {
    double cell_pitch_mm;
    double wall_thickness_mm;
    double robot_half_length_mm;
    double robot_half_width_mm;
    double max_translation_step_mm;
    double max_heading_step_deg;
} NfClearanceConfig;

typedef struct {
    bool clear;
    double first_collision_time_s;
    double robot_x_mm;
    double robot_y_mm;
    double robot_heading_deg;
    uint8_t wall_x;
    uint8_t wall_y;
    NfRouteDirection wall_direction;
    uint32_t sample_count;
} NfClearanceResult;

const char *nf_clearance_status_name(NfClearanceStatus status);

/*
 * Check a nominal turn centre-line against every present maze wall.  The
 * machine is represented by a rectangle and each wall by its physical
 * thickness.  Sampling is made conservative by isotropically inflating the
 * robot rectangle by the maximum possible between-sample corner motion.
 *
 * start_heading_deg uses the mathematical convention: 0 degrees is +X and
 * positive angles rotate towards +Y.  Set turn_left=false to mirror the
 * local turn trajectory for a right turn.
 */
NfClearanceStatus nf_route_turn_clearance(
    const NfRouteMaze *maze,
    const NfClearanceConfig *config,
    const NfTurnSpec *turn,
    const NfTurnPlan *plan,
    double start_x_mm,
    double start_y_mm,
    double start_heading_deg,
    bool turn_left,
    NfClearanceResult *out);

#ifdef __cplusplus
}
#endif

#endif
