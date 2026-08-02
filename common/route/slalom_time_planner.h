#ifndef NIGHTFALL_COMMON_ROUTE_SLALOM_TIME_PLANNER_H
#define NIGHTFALL_COMMON_ROUTE_SLALOM_TIME_PLANNER_H

#include "motion_time.h"
#include "orthogonal_time_planner.h"
#include "route_clearance.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NF_SLALOM_MAX_ACTIONS NF_ROUTE_MAX_ACTIONS
#define NF_SLALOM_MAX_ENDPOINT_CANONICALIZATION_MM 0.001
#define NF_SLALOM_MAX_HEADING_CANONICALIZATION_DEG 1.0e-6

/* Clockwise, in 45 degree increments. */
typedef enum {
    NF_SLALOM_HEADING_NORTH = 0,
    NF_SLALOM_HEADING_NORTH_EAST,
    NF_SLALOM_HEADING_EAST,
    NF_SLALOM_HEADING_SOUTH_EAST,
    NF_SLALOM_HEADING_SOUTH,
    NF_SLALOM_HEADING_SOUTH_WEST,
    NF_SLALOM_HEADING_WEST,
    NF_SLALOM_HEADING_NORTH_WEST,
} NfSlalomHeading8;

typedef enum {
    NF_SLALOM_ACTION_SMALL_90 = 0,
    NF_SLALOM_ACTION_LARGE_90,
    NF_SLALOM_ACTION_LARGE_180,
    NF_SLALOM_ACTION_45_IN,
    NF_SLALOM_ACTION_45_OUT,
    NF_SLALOM_ACTION_V90,
    NF_SLALOM_ACTION_135_IN,
    NF_SLALOM_ACTION_135_OUT,
    NF_SLALOM_ACTION_START_OFFSET,
    NF_SLALOM_ACTION_GOAL_STOP,
    NF_SLALOM_ACTION_KIND_COUNT,
} NfSlalomActionKind;

enum {
    NF_SLALOM_ENABLE_SMALL_90 = 1U << NF_SLALOM_ACTION_SMALL_90,
    NF_SLALOM_ENABLE_LARGE_90 = 1U << NF_SLALOM_ACTION_LARGE_90,
    NF_SLALOM_ENABLE_LARGE_180 = 1U << NF_SLALOM_ACTION_LARGE_180,
    NF_SLALOM_ENABLE_45_IN = 1U << NF_SLALOM_ACTION_45_IN,
    NF_SLALOM_ENABLE_45_OUT = 1U << NF_SLALOM_ACTION_45_OUT,
    NF_SLALOM_ENABLE_V90 = 1U << NF_SLALOM_ACTION_V90,
    NF_SLALOM_ENABLE_135_IN = 1U << NF_SLALOM_ACTION_135_IN,
    NF_SLALOM_ENABLE_135_OUT = 1U << NF_SLALOM_ACTION_135_OUT,
    NF_SLALOM_ENABLE_ALL = (1U << NF_SLALOM_ACTION_START_OFFSET) - 1U,
};

/*
 * The class is deliberately finite.  It records the primitive that supplied
 * the current boundary velocity, so every outgoing edge cost depends only on
 * the current graph state rather than on the complete route history.
 */
typedef enum {
    NF_SLALOM_SPEED_START = 0,
    NF_SLALOM_SPEED_SMALL_90_RIGHT,
    NF_SLALOM_SPEED_SMALL_90_LEFT,
    NF_SLALOM_SPEED_LARGE_90_RIGHT,
    NF_SLALOM_SPEED_LARGE_90_LEFT,
    NF_SLALOM_SPEED_LARGE_180_RIGHT,
    NF_SLALOM_SPEED_LARGE_180_LEFT,
    NF_SLALOM_SPEED_45_IN_RIGHT,
    NF_SLALOM_SPEED_45_IN_LEFT,
    NF_SLALOM_SPEED_45_OUT_RIGHT,
    NF_SLALOM_SPEED_45_OUT_LEFT,
    NF_SLALOM_SPEED_V90_RIGHT,
    NF_SLALOM_SPEED_V90_LEFT,
    NF_SLALOM_SPEED_135_IN_RIGHT,
    NF_SLALOM_SPEED_135_IN_LEFT,
    NF_SLALOM_SPEED_135_OUT_RIGHT,
    NF_SLALOM_SPEED_135_OUT_LEFT,
    NF_SLALOM_SPEED_CLASS_COUNT,
} NfSlalomSpeedClass;

typedef enum {
    NF_SLALOM_GOAL_NONE = 0,
    NF_SLALOM_GOAL_CONNECTOR,
    NF_SLALOM_GOAL_TURN,
} NfSlalomGoalPhase;

typedef enum {
    NF_SLALOM_PLAN_OK = 0,
    NF_SLALOM_PLAN_INVALID_ARGUMENT,
    NF_SLALOM_PLAN_INVALID_MAZE,
    NF_SLALOM_PLAN_INVALID_CONFIG,
    NF_SLALOM_PLAN_UNSUPPORTED_SAFETY,
    NF_SLALOM_PLAN_NO_PATH,
    NF_SLALOM_PLAN_CAPACITY,
    NF_SLALOM_PLAN_OVERFLOW,
} NfSlalomPlanStatus;

typedef struct {
    /* 45 mm half-grid coordinates for a conventional 90 mm maze. */
    int16_t half_x;
    int16_t half_y;
} NfSlalomAnchor;

typedef struct {
    double half_cell_mm;
    /* Controller command distance; logical geometry remains half_cell*sqrt(2). */
    double diagonal_half_command_mm;
    /*
     * Distance from the physical run-start pose to the start-cell centre
     * anchor.  start_velocity_mm_s is the velocity before this offset; the
     * finite START class is its maximum accelerating exit velocity.
     */
    double start_offset_mm;
    double start_velocity_mm_s;
    NfLinearLimits orthogonal;
    NfLinearLimits diagonal;
    NfTurnEnvironment turn_environment;
    NfTurnSpec small_90;
    NfTurnSpec large_90;
    NfTurnSpec large_180;
    NfTurnSpec turn_45_in;
    NfTurnSpec turn_45_out;
    NfTurnSpec v_90;
    NfTurnSpec turn_135_in;
    NfTurnSpec turn_135_out;

    /*
     * PC-only centre-line models that close on the logical half-grid anchors.
     * The named turn fields above remain the timing source: calibrated
     * orthogonal parameters are retained there, while provisional diagonal
     * parameters supply both timing and geometry.  A smooth sin^2 speed
     * correction preserves the configured velocity at both boundaries while
     * fitting the exact-closing path length to the calibrated duration and
     * keeping the resulting speed positive.  Goal
     * entry, required-open replay, clearance, and the following connector
     * therefore share one continuous space-time trajectory.
     */
    NfTurnSpec geometry_turns[NF_SLALOM_ACTION_START_OFFSET];
    uint32_t enabled_actions;

    /*
     * Reject every enabled geometry surrogate that misses its anchor.  These
     * may be made stricter but may not exceed the fixed canonicalization
     * maxima above.
     */
    double orthogonal_anchor_closure_tolerance_mm;
    double diagonal_anchor_closure_tolerance_mm;
    double heading_closure_tolerance_deg;
    /* Used for required-open boundary replay of the nominal centre line. */
    double max_turn_sample_step_mm;

    /* A terminal must retain at least this many connector steps after entry. */
    uint16_t minimum_post_goal_connector_steps;

    /*
     * Optional diagnostic post-check of turns on the selected route.  A
     * collision returns NF_SLALOM_PLAN_UNSUPPORTED_SAFETY; it does not cause
     * Dijkstra to search for the next-best route.  Requiring complete swept
     * clearance is currently unsupported because straight swept volumes are
     * not yet implemented.
     */
    bool check_swept_clearance;
    bool require_swept_clearance;
    NfClearanceConfig clearance;
} NfSlalomPlannerConfig;

typedef struct {
    uint8_t start_x;
    uint8_t start_y;
    NfSlalomHeading8 start_heading;
} NfSlalomPlannerRequest;

typedef struct {
    NfSlalomActionKind kind;
    NfRouteSide side;
    NfSlalomAnchor start_anchor;
    NfSlalomAnchor connector_end_anchor;
    NfSlalomAnchor end_anchor;
    NfSlalomHeading8 start_heading;
    NfSlalomHeading8 end_heading;
    NfSlalomSpeedClass start_speed_class;
    NfSlalomSpeedClass end_speed_class;

    bool connector_is_diagonal;
    uint16_t connector_steps;
    double connector_geometry_distance_mm;
    double connector_command_distance_mm;
    double entry_velocity_mm_s;
    /* Configured command/boundary velocity at both ends of the turn. */
    double turn_velocity_mm_s;
    double exit_velocity_mm_s;
    uint64_t connector_time_us;
    uint64_t turn_time_us;
    uint64_t duration_us;

    /*
     * Global pose at the beginning of a complete turn template.  For
     * START_OFFSET these fields instead hold the physical run-start pose;
     * its three anchor fields all identify the logical start-cell centre.
     */
    double turn_start_x_mm;
    double turn_start_y_mm;
    double turn_start_heading_deg;

    bool required_open_checked;
    bool swept_clearance_checked;
    bool has_goal_cross;
    NfSlalomGoalPhase goal_phase;
    uint8_t goal_x;
    uint8_t goal_y;
    NfSlalomHeading8 goal_cross_heading;
    uint64_t goal_cross_time_us;
} NfSlalomAction;

/*
 * A route plan is bound to the exact NfSlalomPlannerConfig used to build it.
 * Turn actions carry boundary commands and elapsed times, not a duplicate of
 * the geometry seed or sin^2 interior speed profile.  Replay and any future
 * executor must consume the plan together with that same validated config.
 */
typedef struct {
    NfSlalomAction actions[NF_SLALOM_MAX_ACTIONS];
    size_t action_count;
    uint8_t goal_x;
    uint8_t goal_y;
    NfSlalomHeading8 goal_heading;
    uint64_t goal_entry_us;
    uint64_t stop_us;
    double goal_entry_velocity_mm_s;
    bool anchor_closure_validated;
    bool required_open_validated;
    bool swept_clearance_validated;
    uint32_t expanded_states;
    uint32_t relaxed_edges;
} NfSlalomRoutePlan;

typedef struct {
    bool feasible;
    NfSlalomAnchor destination_anchor;
    NfSlalomHeading8 destination_heading;
    uint64_t turn_time_us;
    bool required_open_checked;
} NfSlalomPrimitiveCheck;

typedef struct {
    bool valid;
    size_t action_index;
    uint64_t recomputed_goal_entry_us;
    uint64_t recomputed_stop_us;
    char message[192];
} NfSlalomValidation;

const char *nf_slalom_plan_status_name(NfSlalomPlanStatus status);
const char *nf_slalom_heading_name(NfSlalomHeading8 heading);
const char *nf_slalom_action_name(NfSlalomActionKind kind,
                                  NfRouteSide side);

NfSlalomPlanStatus nf_slalom_time_plan(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    const NfSlalomPlannerRequest *request,
    NfSlalomRoutePlan *out);

NfSlalomPlanStatus nf_slalom_primitive_check(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    NfSlalomAnchor source,
    NfSlalomHeading8 start_heading,
    NfSlalomActionKind kind,
    NfRouteSide side,
    NfSlalomPrimitiveCheck *out);

bool nf_slalom_route_validate(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    const NfSlalomPlannerRequest *request,
    const NfSlalomRoutePlan *plan,
    NfSlalomValidation *validation);

#ifdef __cplusplus
}
#endif

#endif
