#ifndef NIGHTFALL_COMMON_ROUTE_MOTION_TIME_H
#define NIGHTFALL_COMMON_ROUTE_MOTION_TIME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NF_MOTION_LINEAR_MAX_PHASES 5U

typedef enum {
    NF_MOTION_OK = 0,
    NF_MOTION_INVALID_ARGUMENT,
    NF_MOTION_INVALID_LIMITS,
    NF_MOTION_INFEASIBLE,
    NF_MOTION_OVERFLOW,
} NfMotionStatus;

typedef enum {
    NF_LINEAR_PHASE_ACCEL_LOW = 0,
    NF_LINEAR_PHASE_ACCEL_HIGH,
    NF_LINEAR_PHASE_CRUISE,
    NF_LINEAR_PHASE_DECEL_HIGH,
    NF_LINEAR_PHASE_DECEL_LOW,
} NfLinearPhaseKind;

typedef struct {
    double vmax_mm_s;
    double switch_velocity_mm_s;
    double accel_low_mm_s2;
    double accel_high_mm_s2;
} NfLinearLimits;

typedef struct {
    NfLinearPhaseKind kind;
    double distance_mm;
    double duration_s;
    double entry_velocity_mm_s;
    double exit_velocity_mm_s;
    double acceleration_mm_s2;
} NfLinearPhase;

typedef struct {
    NfLinearPhase phases[NF_MOTION_LINEAR_MAX_PHASES];
    size_t phase_count;
    double distance_mm;
    double entry_velocity_mm_s;
    double exit_velocity_mm_s;
    double peak_velocity_mm_s;
    double total_time_s;
} NfLinearPlan;

typedef struct {
    bool enabled;
    double velocity_mm_s;
    double alpha_deg_s2;
    double angle_deg;
    double dist_in_mm;
    double dist_out_mm;
} NfTurnSpec;

typedef struct {
    double omega_cap_deg_s;
    double rounding_scale;
} NfTurnEnvironment;

typedef struct {
    double omega_peak_deg_s;
    double accel_time_s;
    double cruise_time_s;
    double angular_time_s;
    double total_time_s;
    double travel_distance_mm;
    double displacement_forward_mm;
    double displacement_lateral_mm;
} NfTurnPlan;

typedef struct {
    NfLinearPlan full_plan;
    double pre_cross_mm;
    double post_cross_mm;
    double goal_cross_time_s;
    double goal_cross_velocity_mm_s;
} NfGoalTerminalPlan;

const char *nf_motion_status_name(NfMotionStatus status);

NfMotionStatus nf_motion_linear_plan(const NfLinearLimits *limits,
                                     double distance_mm,
                                     double entry_velocity_mm_s,
                                     double exit_velocity_mm_s,
                                     NfLinearPlan *out);

NfMotionStatus nf_motion_accelerating_exit_velocity(
    const NfLinearLimits *limits,
    double distance_mm,
    double entry_velocity_mm_s,
    double *out_exit_velocity_mm_s);

NfMotionStatus nf_motion_linear_time_at_distance(const NfLinearPlan *plan,
                                                  double distance_mm,
                                                  double *out_time_s,
                                                  double *out_velocity_mm_s);

NfMotionStatus nf_motion_turn_plan(const NfTurnSpec *turn,
                                   const NfTurnEnvironment *environment,
                                   NfTurnPlan *out);

NfMotionStatus nf_motion_turn_time_before_end_distance(const NfTurnSpec *turn,
                                                       const NfTurnPlan *plan,
                                                       double remaining_distance_mm,
                                                       double *out_time_s);

/*
 * Treat the turn endpoint as the center anchor of the destination cell and
 * find the latest outside-to-inside crossing of the boundary plane that is
 * remaining_exit_axis_mm behind it along the exit heading.  The raised-cosine
 * angular profile and constant translational velocity are integrated in 2D.
 */
NfMotionStatus nf_motion_turn_exit_boundary_cross(
    const NfTurnSpec *turn,
    const NfTurnPlan *plan,
    double remaining_exit_axis_mm,
    double *out_time_s,
    double *out_lateral_offset_mm);

NfMotionStatus nf_motion_goal_terminal_plan(const NfLinearLimits *limits,
                                            double pre_cross_mm,
                                            double post_cross_mm,
                                            double entry_velocity_mm_s,
                                            NfGoalTerminalPlan *out);

NfMotionStatus nf_motion_seconds_to_us(double seconds, uint64_t *out_us);

#ifdef __cplusplus
}
#endif

#endif
