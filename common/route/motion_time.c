#include "motion_time.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#define NF_MOTION_EPS 1.0e-9
#define NF_MOTION_DISTANCE_SNAP_MM 1.0e-4
#define NF_MOTION_ACCEL_SNAP_RELATIVE 1.0e-6
#define NF_MOTION_PI 3.14159265358979323846264338327950288
#define NF_TURN_INTEGRATION_STEPS 4096U

typedef enum {
    NF_ACCEL_LAYOUT_LOW_ONLY = 0,
    NF_ACCEL_LAYOUT_HIGH_ONLY,
    NF_ACCEL_LAYOUT_TWO_STAGE,
} NfAccelLayout;

static bool nf_finite_nonnegative(double value)
{
    return isfinite(value) && value >= 0.0;
}

static bool nf_motion_nearly_equal(double actual, double expected)
{
    const double scale = fmax(1.0, fmax(fabs(actual), fabs(expected)));
    return isfinite(actual) && isfinite(expected) &&
           fabs(actual - expected) <= (1.0e-9 * scale);
}

static NfMotionStatus nf_accel_layout(const NfLinearLimits *limits,
                                      NfAccelLayout *out_layout)
{
    if (limits == NULL || out_layout == NULL ||
        !isfinite(limits->vmax_mm_s) || limits->vmax_mm_s <= 0.0 ||
        !isfinite(limits->switch_velocity_mm_s) ||
        !isfinite(limits->accel_low_mm_s2) ||
        !isfinite(limits->accel_high_mm_s2)) {
        return NF_MOTION_INVALID_LIMITS;
    }

    if (limits->switch_velocity_mm_s <= 0.0) {
        if (limits->accel_high_mm_s2 <= 0.0) {
            return NF_MOTION_INVALID_LIMITS;
        }
        *out_layout = NF_ACCEL_LAYOUT_HIGH_ONLY;
        return NF_MOTION_OK;
    }
    if (limits->switch_velocity_mm_s >= limits->vmax_mm_s) {
        if (limits->accel_low_mm_s2 <= 0.0) {
            return NF_MOTION_INVALID_LIMITS;
        }
        *out_layout = NF_ACCEL_LAYOUT_LOW_ONLY;
        return NF_MOTION_OK;
    }
    if (limits->accel_low_mm_s2 <= 0.0 || limits->accel_high_mm_s2 <= 0.0) {
        return NF_MOTION_INVALID_LIMITS;
    }
    *out_layout = NF_ACCEL_LAYOUT_TWO_STAGE;
    return NF_MOTION_OK;
}

static void nf_constant_accel_metrics(double velocity_from,
                                      double velocity_to,
                                      double acceleration,
                                      double *distance_mm,
                                      double *duration_s)
{
    *distance_mm += ((velocity_to * velocity_to) -
                     (velocity_from * velocity_from)) / (2.0 * acceleration);
    *duration_s += (velocity_to - velocity_from) / acceleration;
}

static NfMotionStatus nf_speed_change_metrics(const NfLinearLimits *limits,
                                              NfAccelLayout layout,
                                              double velocity_from,
                                              double velocity_to,
                                              double *out_distance_mm,
                                              double *out_duration_s)
{
    double distance_mm = 0.0;
    double duration_s = 0.0;

    if (!nf_finite_nonnegative(velocity_from) ||
        !nf_finite_nonnegative(velocity_to) ||
        velocity_to + NF_MOTION_EPS < velocity_from) {
        return NF_MOTION_INVALID_ARGUMENT;
    }

    if (layout == NF_ACCEL_LAYOUT_LOW_ONLY) {
        nf_constant_accel_metrics(velocity_from, velocity_to,
                                  limits->accel_low_mm_s2,
                                  &distance_mm, &duration_s);
    } else if (layout == NF_ACCEL_LAYOUT_HIGH_ONLY) {
        nf_constant_accel_metrics(velocity_from, velocity_to,
                                  limits->accel_high_mm_s2,
                                  &distance_mm, &duration_s);
    } else {
        const double switch_velocity = limits->switch_velocity_mm_s;
        double current = velocity_from;

        if (current < switch_velocity && velocity_to > current) {
            const double low_end = (velocity_to < switch_velocity) ?
                velocity_to : switch_velocity;
            nf_constant_accel_metrics(current, low_end,
                                      limits->accel_low_mm_s2,
                                      &distance_mm, &duration_s);
            current = low_end;
        }
        if (velocity_to > current) {
            nf_constant_accel_metrics(current, velocity_to,
                                      limits->accel_high_mm_s2,
                                      &distance_mm, &duration_s);
        }
    }

    if (!isfinite(distance_mm) || !isfinite(duration_s)) {
        return NF_MOTION_OVERFLOW;
    }
    *out_distance_mm = distance_mm;
    *out_duration_s = duration_s;
    return NF_MOTION_OK;
}

static NfMotionStatus nf_add_phase(NfLinearPlan *plan,
                                   NfLinearPhaseKind kind,
                                   double velocity_from,
                                   double velocity_to,
                                   double acceleration)
{
    NfLinearPhase *phase;
    double distance_mm;
    double duration_s;
    double total_time_s;

    if (fabs(velocity_to - velocity_from) <= NF_MOTION_EPS) {
        return NF_MOTION_OK;
    }
    if (plan->phase_count >= NF_MOTION_LINEAR_MAX_PHASES ||
        !isfinite(acceleration) || fabs(acceleration) <= NF_MOTION_EPS) {
        return NF_MOTION_INVALID_ARGUMENT;
    }

    distance_mm = ((velocity_to * velocity_to) -
                   (velocity_from * velocity_from)) / (2.0 * acceleration);
    duration_s = (velocity_to - velocity_from) / acceleration;
    total_time_s = plan->total_time_s + duration_s;
    if (!isfinite(distance_mm) || !isfinite(duration_s) ||
        !isfinite(total_time_s)) {
        return NF_MOTION_OVERFLOW;
    }
    if (distance_mm < -NF_MOTION_EPS || duration_s < -NF_MOTION_EPS) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    if (distance_mm < 0.0) {
        distance_mm = 0.0;
    }
    if (duration_s < 0.0) {
        duration_s = 0.0;
    }

    phase = &plan->phases[plan->phase_count++];
    phase->kind = kind;
    phase->entry_velocity_mm_s = velocity_from;
    phase->exit_velocity_mm_s = velocity_to;
    phase->acceleration_mm_s2 = acceleration;
    phase->distance_mm = distance_mm;
    phase->duration_s = duration_s;
    plan->total_time_s = total_time_s;
    return NF_MOTION_OK;
}

static NfMotionStatus nf_add_accel_phases(const NfLinearLimits *limits,
                                          NfAccelLayout layout,
                                          double velocity_from,
                                          double velocity_to,
                                          NfLinearPlan *plan)
{
    if (layout == NF_ACCEL_LAYOUT_LOW_ONLY) {
        return nf_add_phase(plan, NF_LINEAR_PHASE_ACCEL_LOW,
                            velocity_from, velocity_to,
                            limits->accel_low_mm_s2);
    }
    if (layout == NF_ACCEL_LAYOUT_HIGH_ONLY) {
        return nf_add_phase(plan, NF_LINEAR_PHASE_ACCEL_HIGH,
                            velocity_from, velocity_to,
                            limits->accel_high_mm_s2);
    }

    if (velocity_from < limits->switch_velocity_mm_s) {
        const double low_end = (velocity_to < limits->switch_velocity_mm_s) ?
            velocity_to : limits->switch_velocity_mm_s;
        NfMotionStatus status = nf_add_phase(plan, NF_LINEAR_PHASE_ACCEL_LOW,
                                             velocity_from, low_end,
                                             limits->accel_low_mm_s2);
        if (status != NF_MOTION_OK) {
            return status;
        }
        velocity_from = low_end;
    }
    return nf_add_phase(plan, NF_LINEAR_PHASE_ACCEL_HIGH,
                        velocity_from, velocity_to,
                        limits->accel_high_mm_s2);
}

static NfMotionStatus nf_add_decel_phases(const NfLinearLimits *limits,
                                          NfAccelLayout layout,
                                          double velocity_from,
                                          double velocity_to,
                                          NfLinearPlan *plan)
{
    if (layout == NF_ACCEL_LAYOUT_LOW_ONLY) {
        return nf_add_phase(plan, NF_LINEAR_PHASE_DECEL_LOW,
                            velocity_from, velocity_to,
                            -limits->accel_low_mm_s2);
    }
    if (layout == NF_ACCEL_LAYOUT_HIGH_ONLY) {
        return nf_add_phase(plan, NF_LINEAR_PHASE_DECEL_HIGH,
                            velocity_from, velocity_to,
                            -limits->accel_high_mm_s2);
    }

    if (velocity_from > limits->switch_velocity_mm_s) {
        const double high_end = (velocity_to > limits->switch_velocity_mm_s) ?
            velocity_to : limits->switch_velocity_mm_s;
        NfMotionStatus status = nf_add_phase(plan, NF_LINEAR_PHASE_DECEL_HIGH,
                                             velocity_from, high_end,
                                             -limits->accel_high_mm_s2);
        if (status != NF_MOTION_OK) {
            return status;
        }
        velocity_from = high_end;
    }
    return nf_add_phase(plan, NF_LINEAR_PHASE_DECEL_LOW,
                        velocity_from, velocity_to,
                        -limits->accel_low_mm_s2);
}

const char *nf_motion_status_name(NfMotionStatus status)
{
    switch (status) {
    case NF_MOTION_OK: return "ok";
    case NF_MOTION_INVALID_ARGUMENT: return "invalid-argument";
    case NF_MOTION_INVALID_LIMITS: return "invalid-limits";
    case NF_MOTION_INFEASIBLE: return "infeasible";
    case NF_MOTION_OVERFLOW: return "overflow";
    default: return "unknown";
    }
}

NfMotionStatus nf_motion_linear_plan(const NfLinearLimits *limits,
                                     double distance_mm,
                                     double entry_velocity_mm_s,
                                     double exit_velocity_mm_s,
                                     NfLinearPlan *out)
{
    NfLinearLimits snapped_limits;
    const NfLinearLimits *plan_limits = limits;
    NfAccelLayout layout;
    NfMotionStatus status;
    double direct_distance;
    double ignored_time;
    double peak_velocity;
    double accel_distance;
    double decel_distance;
    double cruise_distance;

    if (out == NULL || !nf_finite_nonnegative(distance_mm) ||
        !nf_finite_nonnegative(entry_velocity_mm_s) ||
        !nf_finite_nonnegative(exit_velocity_mm_s)) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));

    status = nf_accel_layout(limits, &layout);
    if (status != NF_MOTION_OK) {
        return status;
    }
    if (entry_velocity_mm_s > limits->vmax_mm_s ||
        exit_velocity_mm_s > limits->vmax_mm_s) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    if (distance_mm <= NF_MOTION_EPS) {
        if (fabs(entry_velocity_mm_s - exit_velocity_mm_s) > NF_MOTION_EPS) {
            return NF_MOTION_INFEASIBLE;
        }
        out->distance_mm = 0.0;
        out->entry_velocity_mm_s = entry_velocity_mm_s;
        out->exit_velocity_mm_s = exit_velocity_mm_s;
        out->peak_velocity_mm_s = entry_velocity_mm_s;
        return NF_MOTION_OK;
    }

    status = nf_speed_change_metrics(limits, layout,
                                     fmin(entry_velocity_mm_s, exit_velocity_mm_s),
                                     fmax(entry_velocity_mm_s, exit_velocity_mm_s),
                                     &direct_distance, &ignored_time);
    if (status != NF_MOTION_OK) {
        return status;
    }
    if (direct_distance > distance_mm + NF_MOTION_EPS) {
        const double excess_mm = direct_distance - distance_mm;
        const double acceleration_scale = direct_distance / distance_mm;

        /*
         * Several firmware tables intentionally describe exact half-cell
         * braking (for example 800 mm/s in 45 mm) with decimal float
         * accelerations.  Their binary/decimal rounding can miss the exact
         * distance by a few nanometres.  Admit only that representation
         * error, and make the returned phases kinematically self-consistent
         * by applying the corresponding sub-ppm acceleration correction.
         */
        if (!isfinite(acceleration_scale) ||
            excess_mm > NF_MOTION_DISTANCE_SNAP_MM ||
            acceleration_scale > 1.0 + NF_MOTION_ACCEL_SNAP_RELATIVE) {
            return NF_MOTION_INFEASIBLE;
        }
        snapped_limits = *limits;
        if (snapped_limits.accel_low_mm_s2 > 0.0) {
            snapped_limits.accel_low_mm_s2 *= acceleration_scale;
        }
        if (snapped_limits.accel_high_mm_s2 > 0.0) {
            snapped_limits.accel_high_mm_s2 *= acceleration_scale;
        }
        status = nf_accel_layout(&snapped_limits, &layout);
        if (status != NF_MOTION_OK) {
            return status;
        }
        plan_limits = &snapped_limits;
        status = nf_speed_change_metrics(
            plan_limits, layout,
            fmin(entry_velocity_mm_s, exit_velocity_mm_s),
            fmax(entry_velocity_mm_s, exit_velocity_mm_s),
            &direct_distance, &ignored_time);
        if (status != NF_MOTION_OK ||
            direct_distance > distance_mm + NF_MOTION_EPS) {
            return (status != NF_MOTION_OK) ? status : NF_MOTION_INFEASIBLE;
        }
    }

    if (direct_distance > distance_mm + NF_MOTION_EPS) {
        return NF_MOTION_INFEASIBLE;
    }

    peak_velocity = plan_limits->vmax_mm_s;
    status = nf_speed_change_metrics(plan_limits, layout, entry_velocity_mm_s,
                                     peak_velocity, &accel_distance,
                                     &ignored_time);
    if (status != NF_MOTION_OK) {
        return status;
    }
    status = nf_speed_change_metrics(plan_limits, layout, exit_velocity_mm_s,
                                     peak_velocity, &decel_distance,
                                     &ignored_time);
    if (status != NF_MOTION_OK) {
        return status;
    }

    if (accel_distance + decel_distance > distance_mm + NF_MOTION_EPS) {
        double low = fmax(entry_velocity_mm_s, exit_velocity_mm_s);
        double high = plan_limits->vmax_mm_s;
        for (unsigned int iteration = 0U; iteration < 80U; iteration++) {
            const double mid = low + (0.5 * (high - low));
            double d0;
            double d1;
            status = nf_speed_change_metrics(plan_limits, layout,
                                             entry_velocity_mm_s, mid,
                                             &d0, &ignored_time);
            if (status != NF_MOTION_OK) {
                return status;
            }
            status = nf_speed_change_metrics(plan_limits, layout,
                                             exit_velocity_mm_s, mid,
                                             &d1, &ignored_time);
            if (status != NF_MOTION_OK) {
                return status;
            }
            if (d0 + d1 <= distance_mm) {
                low = mid;
            } else {
                high = mid;
            }
        }
        peak_velocity = low;
        status = nf_speed_change_metrics(plan_limits, layout,
                                         entry_velocity_mm_s,
                                         peak_velocity, &accel_distance,
                                         &ignored_time);
        if (status != NF_MOTION_OK) {
            return status;
        }
        status = nf_speed_change_metrics(plan_limits, layout,
                                         exit_velocity_mm_s,
                                         peak_velocity, &decel_distance,
                                         &ignored_time);
        if (status != NF_MOTION_OK) {
            return status;
        }
    }

    cruise_distance = distance_mm - accel_distance - decel_distance;
    if (cruise_distance < 0.0 && cruise_distance > -1.0e-6) {
        cruise_distance = 0.0;
    }
    if (cruise_distance < 0.0 || peak_velocity <= 0.0) {
        return NF_MOTION_INFEASIBLE;
    }

    out->distance_mm = distance_mm;
    out->entry_velocity_mm_s = entry_velocity_mm_s;
    out->exit_velocity_mm_s = exit_velocity_mm_s;
    out->peak_velocity_mm_s = peak_velocity;

    status = nf_add_accel_phases(plan_limits, layout, entry_velocity_mm_s,
                                 peak_velocity, out);
    if (status != NF_MOTION_OK) {
        return status;
    }
    if (cruise_distance > NF_MOTION_EPS) {
        NfLinearPhase *phase;
        if (out->phase_count >= NF_MOTION_LINEAR_MAX_PHASES) {
            return NF_MOTION_INVALID_ARGUMENT;
        }
        phase = &out->phases[out->phase_count++];
        phase->kind = NF_LINEAR_PHASE_CRUISE;
        phase->distance_mm = cruise_distance;
        phase->duration_s = cruise_distance / peak_velocity;
        phase->entry_velocity_mm_s = peak_velocity;
        phase->exit_velocity_mm_s = peak_velocity;
        phase->acceleration_mm_s2 = 0.0;
        if (!isfinite(phase->duration_s) ||
            !isfinite(out->total_time_s + phase->duration_s)) {
            out->phase_count--;
            return NF_MOTION_OVERFLOW;
        }
        out->total_time_s += phase->duration_s;
    }
    status = nf_add_decel_phases(plan_limits, layout, peak_velocity,
                                 exit_velocity_mm_s, out);
    if (status != NF_MOTION_OK) {
        return status;
    }
    return NF_MOTION_OK;
}

NfMotionStatus nf_motion_constant_accel_profile(
    const NfLinearLimits *limits,
    double distance_mm,
    double entry_velocity_mm_s,
    double exit_velocity_mm_s,
    NfConstantAccelProfile *out)
{
    NfAccelLayout layout;
    NfMotionStatus status;
    double velocity_sum;
    double acceleration_mm_s2;
    double acceleration_limit_mm_s2;
    double duration_s;

    if (out == NULL || !nf_finite_nonnegative(distance_mm) ||
        !nf_finite_nonnegative(entry_velocity_mm_s) ||
        !nf_finite_nonnegative(exit_velocity_mm_s)) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));

    status = nf_accel_layout(limits, &layout);
    if (status != NF_MOTION_OK) {
        return status;
    }
    if (entry_velocity_mm_s > limits->vmax_mm_s ||
        exit_velocity_mm_s > limits->vmax_mm_s) {
        return NF_MOTION_INVALID_ARGUMENT;
    }

    out->distance_mm = distance_mm;
    out->entry_velocity_mm_s = entry_velocity_mm_s;
    out->exit_velocity_mm_s = exit_velocity_mm_s;

    if (distance_mm <= NF_MOTION_EPS) {
        if (fabs(entry_velocity_mm_s - exit_velocity_mm_s) > NF_MOTION_EPS) {
            return NF_MOTION_INFEASIBLE;
        }
        return NF_MOTION_OK;
    }

    velocity_sum = entry_velocity_mm_s + exit_velocity_mm_s;
    if (!isfinite(velocity_sum)) {
        return NF_MOTION_OVERFLOW;
    }
    if (velocity_sum <= NF_MOTION_EPS) {
        return NF_MOTION_INFEASIBLE;
    }

    duration_s = (2.0 * distance_mm) / velocity_sum;
    acceleration_mm_s2 =
        ((exit_velocity_mm_s - entry_velocity_mm_s) * velocity_sum) /
        (2.0 * distance_mm);
    if (!isfinite(duration_s) || !isfinite(acceleration_mm_s2)) {
        return NF_MOTION_OVERFLOW;
    }

    if (layout == NF_ACCEL_LAYOUT_LOW_ONLY) {
        acceleration_limit_mm_s2 = limits->accel_low_mm_s2;
    } else if (layout == NF_ACCEL_LAYOUT_HIGH_ONLY) {
        acceleration_limit_mm_s2 = limits->accel_high_mm_s2;
    } else {
        const double velocity_low =
            fmin(entry_velocity_mm_s, exit_velocity_mm_s);
        const double velocity_high =
            fmax(entry_velocity_mm_s, exit_velocity_mm_s);

        if (velocity_high <= limits->switch_velocity_mm_s) {
            acceleration_limit_mm_s2 = limits->accel_low_mm_s2;
        } else if (velocity_low >= limits->switch_velocity_mm_s) {
            acceleration_limit_mm_s2 = limits->accel_high_mm_s2;
        } else {
            acceleration_limit_mm_s2 =
                fmin(limits->accel_low_mm_s2, limits->accel_high_mm_s2);
        }
    }

    if (fabs(acceleration_mm_s2) >
        acceleration_limit_mm_s2 * (1.0 + NF_MOTION_ACCEL_SNAP_RELATIVE)) {
        return NF_MOTION_INFEASIBLE;
    }

    out->acceleration_mm_s2 = acceleration_mm_s2;
    out->duration_s = duration_s;
    return NF_MOTION_OK;
}

NfMotionStatus nf_motion_accelerating_exit_velocity(
    const NfLinearLimits *limits,
    double distance_mm,
    double entry_velocity_mm_s,
    double *out_exit_velocity_mm_s)
{
    NfAccelLayout layout;
    double vmax_distance_mm;
    double ignored_time_s;
    double lower_velocity;
    double upper_velocity;
    NfMotionStatus status;

    if (out_exit_velocity_mm_s == NULL ||
        !nf_finite_nonnegative(distance_mm) ||
        !nf_finite_nonnegative(entry_velocity_mm_s)) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    status = nf_accel_layout(limits, &layout);
    if (status != NF_MOTION_OK) {
        return status;
    }
    if (entry_velocity_mm_s > limits->vmax_mm_s + NF_MOTION_EPS) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    if (distance_mm <= NF_MOTION_EPS ||
        entry_velocity_mm_s >= limits->vmax_mm_s - NF_MOTION_EPS) {
        *out_exit_velocity_mm_s = entry_velocity_mm_s;
        return NF_MOTION_OK;
    }

    status = nf_speed_change_metrics(limits, layout, entry_velocity_mm_s,
                                     limits->vmax_mm_s,
                                     &vmax_distance_mm, &ignored_time_s);
    if (status != NF_MOTION_OK) {
        return status;
    }
    if (vmax_distance_mm <= distance_mm + NF_MOTION_EPS) {
        *out_exit_velocity_mm_s = limits->vmax_mm_s;
        return NF_MOTION_OK;
    }

    lower_velocity = entry_velocity_mm_s;
    upper_velocity = limits->vmax_mm_s;
    for (unsigned int i = 0U; i < 96U; i++) {
        const double middle_velocity = lower_velocity +
            (0.5 * (upper_velocity - lower_velocity));
        double required_distance_mm;
        status = nf_speed_change_metrics(limits, layout, entry_velocity_mm_s,
                                         middle_velocity,
                                         &required_distance_mm,
                                         &ignored_time_s);
        if (status != NF_MOTION_OK) {
            return status;
        }
        if (required_distance_mm <= distance_mm) {
            lower_velocity = middle_velocity;
        } else {
            upper_velocity = middle_velocity;
        }
    }
    *out_exit_velocity_mm_s = lower_velocity +
        (0.5 * (upper_velocity - lower_velocity));
    return NF_MOTION_OK;
}

NfMotionStatus nf_motion_linear_time_at_distance(const NfLinearPlan *plan,
                                                  double distance_mm,
                                                  double *out_time_s,
                                                  double *out_velocity_mm_s)
{
    double elapsed_s = 0.0;
    double traversed_mm = 0.0;

    if (plan == NULL || out_time_s == NULL || out_velocity_mm_s == NULL ||
        !isfinite(distance_mm) || distance_mm < -NF_MOTION_EPS ||
        distance_mm > plan->distance_mm + NF_MOTION_EPS) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    if (distance_mm <= NF_MOTION_EPS) {
        *out_time_s = 0.0;
        *out_velocity_mm_s = plan->entry_velocity_mm_s;
        return NF_MOTION_OK;
    }
    if (fabs(distance_mm - plan->distance_mm) <= NF_MOTION_EPS) {
        *out_time_s = plan->total_time_s;
        *out_velocity_mm_s = plan->exit_velocity_mm_s;
        return NF_MOTION_OK;
    }

    for (size_t i = 0U; i < plan->phase_count; i++) {
        const NfLinearPhase *phase = &plan->phases[i];
        if (distance_mm <= traversed_mm + phase->distance_mm + NF_MOTION_EPS) {
            double local_mm = distance_mm - traversed_mm;
            double local_s;
            double velocity;
            if (local_mm < 0.0) {
                local_mm = 0.0;
            }
            if (local_mm > phase->distance_mm) {
                local_mm = phase->distance_mm;
            }
            if (fabs(phase->acceleration_mm_s2) <= NF_MOTION_EPS) {
                if (phase->entry_velocity_mm_s <= 0.0) {
                    return NF_MOTION_INFEASIBLE;
                }
                velocity = phase->entry_velocity_mm_s;
                local_s = local_mm / velocity;
            } else {
                double velocity_sq =
                    (phase->entry_velocity_mm_s * phase->entry_velocity_mm_s) +
                    (2.0 * phase->acceleration_mm_s2 * local_mm);
                if (velocity_sq < 0.0 && velocity_sq > -1.0e-6) {
                    velocity_sq = 0.0;
                }
                if (velocity_sq < 0.0) {
                    return NF_MOTION_INFEASIBLE;
                }
                velocity = sqrt(velocity_sq);
                local_s = (velocity - phase->entry_velocity_mm_s) /
                          phase->acceleration_mm_s2;
            }
            *out_time_s = elapsed_s + local_s;
            *out_velocity_mm_s = velocity;
            return NF_MOTION_OK;
        }
        elapsed_s += phase->duration_s;
        traversed_mm += phase->distance_mm;
    }

    if (fabs(distance_mm - plan->distance_mm) <= 1.0e-6) {
        *out_time_s = plan->total_time_s;
        *out_velocity_mm_s = plan->exit_velocity_mm_s;
        return NF_MOTION_OK;
    }
    return NF_MOTION_INVALID_ARGUMENT;
}

static double nf_turn_heading_deg_at_time(const NfTurnSpec *turn,
                                          const NfTurnPlan *plan,
                                          double time_s)
{
    const double accel_time_s = plan->accel_time_s;
    const double cruise_time_s = plan->cruise_time_s;
    const double omega = plan->omega_peak_deg_s;

    if (time_s <= 0.0) {
        return 0.0;
    }
    if (time_s < accel_time_s) {
        const double x = NF_MOTION_PI * time_s / accel_time_s;
        return 0.5 * omega *
            (time_s - ((accel_time_s / NF_MOTION_PI) * sin(x)));
    }
    if (time_s < accel_time_s + cruise_time_s) {
        return (0.5 * omega * accel_time_s) +
               (omega * (time_s - accel_time_s));
    }
    if (time_s < plan->angular_time_s) {
        const double decel_time_s = time_s - accel_time_s - cruise_time_s;
        const double x = NF_MOTION_PI * decel_time_s / accel_time_s;
        return (0.5 * omega * accel_time_s) +
               (omega * cruise_time_s) +
               (0.5 * omega *
                (decel_time_s +
                 ((accel_time_s / NF_MOTION_PI) * sin(x))));
    }
    return turn->angle_deg;
}

static void nf_turn_integrate_projection_interval(
    const NfTurnSpec *turn,
    const NfTurnPlan *plan,
    double start_time_s,
    double end_time_s,
    double axis_heading_deg,
    double *out_parallel_mm,
    double *out_lateral_mm)
{
    const double middle_time_s = 0.5 * (start_time_s + end_time_s);
    const double scale = turn->velocity_mm_s *
                         (end_time_s - start_time_s) / 6.0;
    const double start_angle =
        (nf_turn_heading_deg_at_time(turn, plan, start_time_s) -
         axis_heading_deg) * (NF_MOTION_PI / 180.0);
    const double middle_angle =
        (nf_turn_heading_deg_at_time(turn, plan, middle_time_s) -
         axis_heading_deg) * (NF_MOTION_PI / 180.0);
    const double end_angle =
        (nf_turn_heading_deg_at_time(turn, plan, end_time_s) -
         axis_heading_deg) * (NF_MOTION_PI / 180.0);

    *out_parallel_mm = scale *
        (cos(start_angle) + (4.0 * cos(middle_angle)) + cos(end_angle));
    *out_lateral_mm = scale *
        (sin(start_angle) + (4.0 * sin(middle_angle)) + sin(end_angle));
}

static void nf_turn_integrate_projection_steps(const NfTurnSpec *turn,
                                               const NfTurnPlan *plan,
                                               double start_time_s,
                                               double end_time_s,
                                               double axis_heading_deg,
                                               unsigned int steps,
                                               double *out_parallel_mm,
                                               double *out_lateral_mm)
{
    double parallel_mm = 0.0;
    double lateral_mm = 0.0;
    double interval_start = start_time_s;

    for (unsigned int i = 0U; i < steps; i++) {
        const double interval_end = start_time_s +
            ((end_time_s - start_time_s) * (double)(i + 1U) /
             (double)steps);
        double interval_parallel;
        double interval_lateral;
        nf_turn_integrate_projection_interval(turn, plan,
                                              interval_start, interval_end,
                                              axis_heading_deg,
                                              &interval_parallel,
                                              &interval_lateral);
        parallel_mm += interval_parallel;
        lateral_mm += interval_lateral;
        interval_start = interval_end;
    }
    *out_parallel_mm = parallel_mm;
    *out_lateral_mm = lateral_mm;
}

static void nf_turn_integrate_projection(const NfTurnSpec *turn,
                                         const NfTurnPlan *plan,
                                         double start_time_s,
                                         double end_time_s,
                                         double axis_heading_deg,
                                         double *out_parallel_mm,
                                         double *out_lateral_mm)
{
    nf_turn_integrate_projection_steps(
        turn, plan, start_time_s, end_time_s, axis_heading_deg,
        NF_TURN_INTEGRATION_STEPS, out_parallel_mm, out_lateral_mm);
}

static double nf_turn_time_at_heading(const NfTurnSpec *turn,
                                      const NfTurnPlan *plan,
                                      double target_heading_deg)
{
    double lower_s = 0.0;
    double upper_s = plan->angular_time_s;

    if (target_heading_deg <= 0.0) {
        return 0.0;
    }
    if (target_heading_deg >= turn->angle_deg) {
        return plan->angular_time_s;
    }
    for (unsigned int i = 0U; i < 64U; i++) {
        const double middle_s = 0.5 * (lower_s + upper_s);
        if (nf_turn_heading_deg_at_time(turn, plan, middle_s) <
            target_heading_deg) {
            lower_s = middle_s;
        } else {
            upper_s = middle_s;
        }
    }
    return 0.5 * (lower_s + upper_s);
}

NfMotionStatus nf_motion_turn_plan(const NfTurnSpec *turn,
                                   const NfTurnEnvironment *environment,
                                   NfTurnPlan *out)
{
    NfTurnPlan plan;
    double omega_peak;
    double rounding_scale;
    double t_acc;
    double t_cruise;

    if (turn == NULL || environment == NULL || out == NULL) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    memset(&plan, 0, sizeof(plan));
    if (!turn->enabled) {
        return NF_MOTION_INFEASIBLE;
    }
    if (!isfinite(turn->velocity_mm_s) || turn->velocity_mm_s <= 0.0 ||
        !isfinite(turn->alpha_deg_s2) || turn->alpha_deg_s2 <= 0.0 ||
        !isfinite(turn->angle_deg) || turn->angle_deg <= 0.0 ||
        !nf_finite_nonnegative(turn->dist_in_mm) ||
        !nf_finite_nonnegative(turn->dist_out_mm) ||
        !isfinite(environment->omega_cap_deg_s) ||
        !isfinite(environment->rounding_scale) ||
        environment->rounding_scale <= 0.0) {
        return NF_MOTION_INVALID_ARGUMENT;
    }

    /* Keep the mathematically finite sqrt(alpha * angle) representable. */
    omega_peak = sqrt(turn->alpha_deg_s2) *
                 sqrt((2.0 / 3.0) * turn->angle_deg);
    if (!isfinite(omega_peak)) {
        return NF_MOTION_OVERFLOW;
    }
    if (environment->omega_cap_deg_s > 0.0 &&
        omega_peak > environment->omega_cap_deg_s) {
        omega_peak = environment->omega_cap_deg_s;
    }
    if (omega_peak <= 0.0) {
        return NF_MOTION_INFEASIBLE;
    }

    rounding_scale = environment->rounding_scale;
    if (rounding_scale < 0.1) {
        rounding_scale = 0.1;
    }
    t_acc = (omega_peak / turn->alpha_deg_s2) * rounding_scale;
    if (!isfinite(t_acc) || t_acc <= 0.0) {
        return NF_MOTION_OVERFLOW;
    }
    t_cruise = (turn->angle_deg / omega_peak) - t_acc;
    if (t_cruise < 0.0) {
        omega_peak = turn->angle_deg / t_acc;
        if (environment->omega_cap_deg_s > 0.0 &&
            omega_peak > environment->omega_cap_deg_s) {
            omega_peak = environment->omega_cap_deg_s;
        }
        if (!isfinite(omega_peak) || omega_peak <= 0.0) {
            return NF_MOTION_OVERFLOW;
        }
        t_cruise = (turn->angle_deg / omega_peak) - t_acc;
        if (t_cruise < 0.0) {
            t_cruise = 0.0;
        }
    }

    plan.omega_peak_deg_s = omega_peak;
    plan.accel_time_s = t_acc;
    plan.cruise_time_s = t_cruise;
    plan.angular_time_s = (2.0 * t_acc) + t_cruise;
    plan.total_time_s = (turn->dist_in_mm / turn->velocity_mm_s) +
                        plan.angular_time_s +
                        (turn->dist_out_mm / turn->velocity_mm_s);
    plan.travel_distance_mm = turn->dist_in_mm + turn->dist_out_mm +
                              (turn->velocity_mm_s * plan.angular_time_s);
    if (!isfinite(plan.angular_time_s) ||
        !isfinite(plan.total_time_s) ||
        !isfinite(plan.travel_distance_mm)) {
        return NF_MOTION_OVERFLOW;
    }
    {
        double angular_forward_mm;
        double angular_lateral_mm;
        const double angle_rad = turn->angle_deg * (NF_MOTION_PI / 180.0);
        nf_turn_integrate_projection(turn, &plan, 0.0, plan.angular_time_s,
                                     0.0, &angular_forward_mm,
                                     &angular_lateral_mm);
        plan.displacement_forward_mm = turn->dist_in_mm + angular_forward_mm +
                                       (turn->dist_out_mm * cos(angle_rad));
        plan.displacement_lateral_mm = angular_lateral_mm +
                                       (turn->dist_out_mm * sin(angle_rad));
        if (!isfinite(plan.displacement_forward_mm) ||
            !isfinite(plan.displacement_lateral_mm)) {
            return NF_MOTION_OVERFLOW;
        }
    }
    *out = plan;
    return NF_MOTION_OK;
}

NfMotionStatus nf_motion_turn_pose_at_time(const NfTurnSpec *turn,
                                           const NfTurnPlan *plan,
                                           double elapsed_s,
                                           NfTurnPose *out)
{
    double angular_start_s;
    double angular_end_s;
    double angle_rad;
    double expected_angular_time_s;
    double expected_total_time_s;
    double expected_travel_distance_mm;

    if (turn == NULL || plan == NULL || out == NULL ||
        !isfinite(elapsed_s) || elapsed_s < -NF_MOTION_EPS ||
        elapsed_s > plan->total_time_s + NF_MOTION_EPS ||
        !isfinite(turn->velocity_mm_s) || turn->velocity_mm_s <= 0.0 ||
        !isfinite(turn->alpha_deg_s2) || turn->alpha_deg_s2 <= 0.0 ||
        !nf_finite_nonnegative(turn->dist_in_mm) ||
        !nf_finite_nonnegative(turn->dist_out_mm) ||
        !isfinite(turn->angle_deg) || turn->angle_deg <= 0.0 ||
        !isfinite(plan->omega_peak_deg_s) || plan->omega_peak_deg_s <= 0.0 ||
        !isfinite(plan->accel_time_s) || plan->accel_time_s <= 0.0 ||
        !nf_finite_nonnegative(plan->cruise_time_s) ||
        !isfinite(plan->angular_time_s) || plan->angular_time_s <= 0.0 ||
        !isfinite(plan->total_time_s) || plan->total_time_s <= 0.0 ||
        !isfinite(plan->travel_distance_mm) ||
        plan->travel_distance_mm <= 0.0 ||
        !isfinite(plan->displacement_forward_mm) ||
        !isfinite(plan->displacement_lateral_mm)) {
        return NF_MOTION_INVALID_ARGUMENT;
    }

    expected_angular_time_s =
        (2.0 * plan->accel_time_s) + plan->cruise_time_s;
    expected_total_time_s =
        (turn->dist_in_mm / turn->velocity_mm_s) +
        expected_angular_time_s +
        (turn->dist_out_mm / turn->velocity_mm_s);
    expected_travel_distance_mm =
        turn->dist_in_mm + turn->dist_out_mm +
        (turn->velocity_mm_s * expected_angular_time_s);
    if (!nf_motion_nearly_equal(plan->angular_time_s,
                                expected_angular_time_s) ||
        !nf_motion_nearly_equal(plan->total_time_s,
                                expected_total_time_s) ||
        !nf_motion_nearly_equal(plan->travel_distance_mm,
                                expected_travel_distance_mm) ||
        !nf_motion_nearly_equal(
            turn->angle_deg,
            plan->omega_peak_deg_s *
                (plan->accel_time_s + plan->cruise_time_s))) {
        return NF_MOTION_INVALID_ARGUMENT;
    }

    if (elapsed_s < 0.0) {
        elapsed_s = 0.0;
    }
    if (elapsed_s > plan->total_time_s) {
        elapsed_s = plan->total_time_s;
    }
    memset(out, 0, sizeof(*out));

    angular_start_s = turn->dist_in_mm / turn->velocity_mm_s;
    angular_end_s = angular_start_s + plan->angular_time_s;
    if (elapsed_s <= angular_start_s) {
        out->forward_mm = turn->velocity_mm_s * elapsed_s;
        return NF_MOTION_OK;
    }

    {
        const double angular_elapsed_s =
            fmin(elapsed_s - angular_start_s, plan->angular_time_s);
        double angular_forward_mm;
        double angular_lateral_mm;
        nf_turn_integrate_projection(turn, plan, 0.0, angular_elapsed_s,
                                     0.0, &angular_forward_mm,
                                     &angular_lateral_mm);
        out->forward_mm = turn->dist_in_mm + angular_forward_mm;
        out->lateral_mm = angular_lateral_mm;
        out->heading_deg = nf_turn_heading_deg_at_time(
            turn, plan, angular_elapsed_s);
    }

    if (elapsed_s <= angular_end_s) {
        return NF_MOTION_OK;
    }

    angle_rad = turn->angle_deg * (NF_MOTION_PI / 180.0);
    {
        const double exit_distance_mm =
            turn->velocity_mm_s * (elapsed_s - angular_end_s);
        out->forward_mm += exit_distance_mm * cos(angle_rad);
        out->lateral_mm += exit_distance_mm * sin(angle_rad);
        out->heading_deg = turn->angle_deg;
    }
    return NF_MOTION_OK;
}

NfMotionStatus nf_motion_turn_pose_uniform(const NfTurnSpec *turn,
                                           const NfTurnPlan *plan,
                                           size_t interval_count,
                                           NfTurnPose *poses,
                                           size_t pose_capacity)
{
    NfTurnPose validation_pose;
    double angular_start_s;
    double angular_end_s;
    double forward_mm = 0.0;
    double lateral_mm = 0.0;
    double previous_time_s = 0.0;

    if (turn == NULL || plan == NULL || poses == NULL ||
        interval_count == 0U || interval_count == SIZE_MAX ||
        pose_capacity < interval_count + 1U) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    angular_start_s = turn->dist_in_mm / turn->velocity_mm_s;
    angular_end_s = angular_start_s + plan->angular_time_s;
    if (nf_motion_turn_pose_at_time(turn, plan, 0.0, &validation_pose) !=
        NF_MOTION_OK) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    poses[0] = validation_pose;

    for (size_t i = 1U; i <= interval_count; i++) {
        const double elapsed_s =
            plan->total_time_s * (double)i / (double)interval_count;
        double cursor_s = previous_time_s;

        if (cursor_s < angular_start_s) {
            const double straight_end_s = fmin(elapsed_s, angular_start_s);
            if (straight_end_s > cursor_s) {
                forward_mm += turn->velocity_mm_s *
                              (straight_end_s - cursor_s);
                cursor_s = straight_end_s;
            }
        }
        if (cursor_s < elapsed_s && cursor_s < angular_end_s &&
            elapsed_s > angular_start_s) {
            const double overlap_start_s = fmax(cursor_s, angular_start_s);
            const double overlap_end_s = fmin(elapsed_s, angular_end_s);
            if (overlap_end_s > overlap_start_s) {
                const double local_start_s =
                    overlap_start_s - angular_start_s;
                const double local_end_s = overlap_end_s - angular_start_s;
                const double fraction =
                    (local_end_s - local_start_s) / plan->angular_time_s;
                unsigned int steps = (unsigned int)ceil(
                    fraction * (double)NF_TURN_INTEGRATION_STEPS);
                double delta_forward_mm;
                double delta_lateral_mm;
                if (steps == 0U) {
                    steps = 1U;
                }
                nf_turn_integrate_projection_steps(
                    turn, plan, local_start_s, local_end_s, 0.0, steps,
                    &delta_forward_mm, &delta_lateral_mm);
                forward_mm += delta_forward_mm;
                lateral_mm += delta_lateral_mm;
                cursor_s = overlap_end_s;
            }
        }
        if (cursor_s < elapsed_s) {
            const double exit_distance_mm =
                turn->velocity_mm_s * (elapsed_s - cursor_s);
            const double angle_rad =
                turn->angle_deg * (NF_MOTION_PI / 180.0);
            forward_mm += exit_distance_mm * cos(angle_rad);
            lateral_mm += exit_distance_mm * sin(angle_rad);
        }

        poses[i].forward_mm = forward_mm;
        poses[i].lateral_mm = lateral_mm;
        if (elapsed_s <= angular_start_s) {
            poses[i].heading_deg = 0.0;
        } else if (elapsed_s >= angular_end_s) {
            poses[i].heading_deg = turn->angle_deg;
        } else {
            poses[i].heading_deg = nf_turn_heading_deg_at_time(
                turn, plan, elapsed_s - angular_start_s);
        }
        previous_time_s = elapsed_s;
    }
    return NF_MOTION_OK;
}

NfMotionStatus nf_motion_turn_time_before_end_distance(const NfTurnSpec *turn,
                                                       const NfTurnPlan *plan,
                                                       double remaining_distance_mm,
                                                       double *out_time_s)
{
    if (turn == NULL || plan == NULL || out_time_s == NULL ||
        !nf_finite_nonnegative(remaining_distance_mm) ||
        turn->velocity_mm_s <= 0.0 ||
        remaining_distance_mm > plan->travel_distance_mm + NF_MOTION_EPS) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    *out_time_s = plan->total_time_s -
                  (remaining_distance_mm / turn->velocity_mm_s);
    if (*out_time_s < -NF_MOTION_EPS) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    if (*out_time_s < 0.0) {
        *out_time_s = 0.0;
    }
    return NF_MOTION_OK;
}

NfMotionStatus nf_motion_turn_exit_boundary_cross(
    const NfTurnSpec *turn,
    const NfTurnPlan *plan,
    double remaining_exit_axis_mm,
    double *out_time_s,
    double *out_lateral_offset_mm)
{
    double target_angular_projection_mm;
    double minimum_time_s;
    double previous_time_s;
    double remaining_parallel_mm = 0.0;
    double remaining_lateral_mm = 0.0;

    if (turn == NULL || plan == NULL || out_time_s == NULL ||
        out_lateral_offset_mm == NULL ||
        !nf_finite_nonnegative(remaining_exit_axis_mm) ||
        !isfinite(plan->accel_time_s) || plan->accel_time_s <= 0.0 ||
        !nf_finite_nonnegative(plan->cruise_time_s) ||
        !isfinite(plan->angular_time_s) || plan->angular_time_s <= 0.0 ||
        !isfinite(plan->total_time_s) || plan->total_time_s <= 0.0 ||
        !isfinite(turn->velocity_mm_s) || turn->velocity_mm_s <= 0.0 ||
        !isfinite(turn->angle_deg) || turn->angle_deg <= 0.0 ||
        !nf_finite_nonnegative(turn->dist_in_mm) ||
        !nf_finite_nonnegative(turn->dist_out_mm)) {
        return NF_MOTION_INVALID_ARGUMENT;
    }

    if (remaining_exit_axis_mm <= turn->dist_out_mm + NF_MOTION_EPS) {
        const double clamped_remaining =
            (remaining_exit_axis_mm > turn->dist_out_mm) ?
                turn->dist_out_mm : remaining_exit_axis_mm;
        *out_time_s = plan->total_time_s -
                      (clamped_remaining / turn->velocity_mm_s);
        *out_lateral_offset_mm = 0.0;
        return NF_MOTION_OK;
    }

    target_angular_projection_mm = remaining_exit_axis_mm - turn->dist_out_mm;
    minimum_time_s = nf_turn_time_at_heading(
        turn, plan, fmax(0.0, turn->angle_deg - 90.0));
    previous_time_s = plan->angular_time_s;

    for (unsigned int i = 1U; i <= NF_TURN_INTEGRATION_STEPS; i++) {
        double interval_start_s = plan->angular_time_s *
            (double)(NF_TURN_INTEGRATION_STEPS - i) /
            (double)NF_TURN_INTEGRATION_STEPS;
        double interval_parallel_mm;
        double interval_lateral_mm;
        bool last_interval = false;

        if (interval_start_s <= minimum_time_s) {
            interval_start_s = minimum_time_s;
            last_interval = true;
        }
        nf_turn_integrate_projection_interval(
            turn, plan, interval_start_s, previous_time_s, turn->angle_deg,
            &interval_parallel_mm, &interval_lateral_mm);

        if (remaining_parallel_mm + interval_parallel_mm + NF_MOTION_EPS >=
            target_angular_projection_mm) {
            double lower_s = interval_start_s;
            double upper_s = previous_time_s;
            double root_parallel_mm = 0.0;
            double root_lateral_mm = 0.0;

            for (unsigned int iteration = 0U; iteration < 64U; iteration++) {
                const double middle_s = 0.5 * (lower_s + upper_s);
                double middle_parallel_mm;
                double middle_lateral_mm;
                nf_turn_integrate_projection_interval(
                    turn, plan, middle_s, previous_time_s, turn->angle_deg,
                    &middle_parallel_mm, &middle_lateral_mm);
                if (remaining_parallel_mm + middle_parallel_mm >=
                    target_angular_projection_mm) {
                    lower_s = middle_s;
                } else {
                    upper_s = middle_s;
                }
            }
            {
                const double root_s = 0.5 * (lower_s + upper_s);
                const double relative_heading_rad =
                    (nf_turn_heading_deg_at_time(turn, plan, root_s) -
                     turn->angle_deg) * (NF_MOTION_PI / 180.0);
                if (cos(relative_heading_rad) <= 1.0e-8) {
                    return NF_MOTION_INFEASIBLE;
                }
                nf_turn_integrate_projection_interval(
                    turn, plan, root_s, previous_time_s, turn->angle_deg,
                    &root_parallel_mm, &root_lateral_mm);
                *out_time_s = (turn->dist_in_mm / turn->velocity_mm_s) + root_s;
                *out_lateral_offset_mm =
                    -(remaining_lateral_mm + root_lateral_mm);
            }
            return NF_MOTION_OK;
        }

        remaining_parallel_mm += interval_parallel_mm;
        remaining_lateral_mm += interval_lateral_mm;
        previous_time_s = interval_start_s;
        if (last_interval) {
            break;
        }
    }
    return NF_MOTION_INFEASIBLE;
}

NfMotionStatus nf_motion_goal_terminal_plan(const NfLinearLimits *limits,
                                            double pre_cross_mm,
                                            double post_cross_mm,
                                            double entry_velocity_mm_s,
                                            NfGoalTerminalPlan *out)
{
    NfMotionStatus status;

    if (out == NULL || !nf_finite_nonnegative(pre_cross_mm) ||
        !nf_finite_nonnegative(post_cross_mm)) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    if (pre_cross_mm > DBL_MAX - post_cross_mm) {
        return NF_MOTION_OVERFLOW;
    }
    status = nf_motion_linear_plan(limits, pre_cross_mm + post_cross_mm,
                                   entry_velocity_mm_s, 0.0,
                                   &out->full_plan);
    if (status != NF_MOTION_OK) {
        return status;
    }
    out->pre_cross_mm = pre_cross_mm;
    out->post_cross_mm = post_cross_mm;
    return nf_motion_linear_time_at_distance(&out->full_plan, pre_cross_mm,
                                             &out->goal_cross_time_s,
                                             &out->goal_cross_velocity_mm_s);
}

NfMotionStatus nf_motion_seconds_to_us(double seconds, uint64_t *out_us)
{
    long double rounded;
    long double scaled;

    if (out_us == NULL || !nf_finite_nonnegative(seconds)) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    scaled = (long double)seconds * 1000000.0L;
    if (!isfinite(scaled) ||
        scaled + 0.5L >= 18446744073709551616.0L) {
        return NF_MOTION_OVERFLOW;
    }
    rounded = floorl(scaled + 0.5L);
    if (seconds > 0.0 && rounded < 1.0L) {
        return NF_MOTION_INVALID_ARGUMENT;
    }
    *out_us = (uint64_t)rounded;
    return NF_MOTION_OK;
}
