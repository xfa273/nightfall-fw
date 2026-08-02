#include "slalom_profile_baseline.h"
#include "shortest_run_params.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#define NF_SEED_CLOSURE_TOLERANCE_MM 0.001
#define NF_BASELINE_REGRESSION_TOLERANCE_MM 1.0e-6
#define NF_MIRROR_TOLERANCE_MM 1.0e-9

static unsigned int g_checks;
static unsigned int g_failures;

static const ShortestRunModeParams_t *nf_compiled_mode(size_t profile_index)
{
    switch (profile_index) {
    case 0U:
    case 1U:
        return &shortestRunModeParams2;
    case 2U:
        return &shortestRunModeParams3;
    case 3U:
        return &shortestRunModeParams4;
    case 4U:
        return &shortestRunModeParams5;
    default:
        return NULL;
    }
}

static NfCurrentPrimitive nf_compiled_primitive(
    const ShortestRunModeParams_t *mode,
    NfPrimitiveId primitive)
{
    NfCurrentPrimitive actual = {0};

    switch (primitive) {
    case NF_PRIMITIVE_SMALL_90:
        actual.velocity_mm_s = mode->velocity_turn90;
        actual.alpha_deg_s2 = mode->alpha_turn90;
        actual.dist_in_mm = mode->dist_offset_in;
        actual.dist_out_mm = mode->dist_offset_out;
        break;
    case NF_PRIMITIVE_LARGE_90:
        actual.velocity_mm_s = mode->velocity_l_turn_90;
        actual.alpha_deg_s2 = mode->alpha_l_turn_90;
        actual.dist_in_mm = mode->dist_l_turn_in_90;
        actual.dist_out_mm = mode->dist_l_turn_out_90;
        break;
    case NF_PRIMITIVE_LARGE_180:
        actual.velocity_mm_s = mode->velocity_l_turn_180;
        actual.alpha_deg_s2 = mode->alpha_l_turn_180;
        actual.dist_in_mm = mode->dist_l_turn_in_180;
        actual.dist_out_mm = mode->dist_l_turn_out_180;
        break;
    case NF_PRIMITIVE_45_IN:
        actual.velocity_mm_s = mode->velocity_turn45in;
        actual.alpha_deg_s2 = mode->alpha_turn45in;
        actual.dist_in_mm = mode->dist_turn45in_in;
        actual.dist_out_mm = mode->dist_turn45in_out;
        break;
    case NF_PRIMITIVE_45_OUT:
        actual.velocity_mm_s = mode->velocity_turn45out;
        actual.alpha_deg_s2 = mode->alpha_turn45out;
        actual.dist_in_mm = mode->dist_turn45out_in;
        actual.dist_out_mm = mode->dist_turn45out_out;
        break;
    case NF_PRIMITIVE_V90:
        actual.velocity_mm_s = mode->velocity_turnV90;
        actual.alpha_deg_s2 = mode->alpha_turnV90;
        actual.dist_in_mm = mode->dist_turnV90_in;
        actual.dist_out_mm = mode->dist_turnV90_out;
        break;
    case NF_PRIMITIVE_135_IN:
        actual.velocity_mm_s = mode->velocity_turn135in;
        actual.alpha_deg_s2 = mode->alpha_turn135in;
        actual.dist_in_mm = mode->dist_turn135in_in;
        actual.dist_out_mm = mode->dist_turn135in_out;
        break;
    case NF_PRIMITIVE_135_OUT:
        actual.velocity_mm_s = mode->velocity_turn135out;
        actual.alpha_deg_s2 = mode->alpha_turn135out;
        actual.dist_in_mm = mode->dist_turn135out_in;
        actual.dist_out_mm = mode->dist_turn135out_out;
        break;
    case NF_PRIMITIVE_COUNT:
    default:
        return actual;
    }
    actual.available = actual.velocity_mm_s > 0.0 &&
                       actual.alpha_deg_s2 > 0.0;
    return actual;
}

static void nf_check(bool condition, const char *profile, const char *primitive,
                     const char *message)
{
    g_checks++;
    if (!condition) {
        fprintf(stderr, "%s/%s: %s\n", profile, primitive, message);
        g_failures++;
    }
}

static void nf_check_close(double actual, double expected, double tolerance,
                           const char *profile, const char *primitive,
                           const char *message)
{
    nf_check(isfinite(actual) && fabs(actual - expected) <= tolerance,
             profile, primitive, message);
}

static NfTurnSpec nf_make_turn(double angle_deg, double velocity_mm_s,
                               double alpha_deg_s2, double dist_in_mm,
                               double dist_out_mm)
{
    const NfTurnSpec turn = {
        .enabled = true,
        .velocity_mm_s = velocity_mm_s,
        .alpha_deg_s2 = alpha_deg_s2,
        .angle_deg = angle_deg,
        .dist_in_mm = dist_in_mm,
        .dist_out_mm = dist_out_mm,
    };
    return turn;
}

static void nf_check_mirror(const char *profile_name,
                            const NfPrimitiveGeometry *geometry,
                            const NfTurnPlan *plan,
                            double tolerance_mm)
{
    const double left_residual_forward =
        plan->displacement_forward_mm - geometry->target_forward_mm;
    const double left_residual_lateral =
        plan->displacement_lateral_mm - geometry->target_lateral_mm;
    const double right_forward_mm = plan->displacement_forward_mm;
    const double right_lateral_mm = -plan->displacement_lateral_mm;
    const double right_target_forward_mm = geometry->target_forward_mm;
    const double right_target_lateral_mm = -geometry->target_lateral_mm;
    const double right_residual_forward =
        right_forward_mm - right_target_forward_mm;
    const double right_residual_lateral =
        right_lateral_mm - right_target_lateral_mm;

    nf_check_close(right_forward_mm, plan->displacement_forward_mm,
                   tolerance_mm, profile_name, geometry->name,
                   "right/left forward mirror mismatch");
    nf_check_close(right_lateral_mm, -plan->displacement_lateral_mm,
                   tolerance_mm, profile_name, geometry->name,
                   "right/left lateral mirror mismatch");
    nf_check_close(right_residual_forward, left_residual_forward,
                   tolerance_mm, profile_name, geometry->name,
                   "right/left forward residual mismatch");
    nf_check_close(right_residual_lateral, -left_residual_lateral,
                   tolerance_mm, profile_name, geometry->name,
                   "right/left lateral residual mismatch");
    nf_check_close(hypot(right_residual_forward, right_residual_lateral),
                   hypot(left_residual_forward, left_residual_lateral),
                   tolerance_mm, profile_name, geometry->name,
                   "right/left residual norm mismatch");
}

static void nf_audit_compiled_source(size_t profile_index,
                                     const NfAuditProfile *profile)
{
    const ShortestRunModeParams_t *mode = nf_compiled_mode(profile_index);

    nf_check(mode != NULL, profile->name, "compiled-source",
             "profile has no compiled shortest-run source mapping");
    if (mode == NULL) {
        return;
    }
    for (size_t i = 0U; i < NF_PRIMITIVE_COUNT; i++) {
        const NfCurrentPrimitive actual = nf_compiled_primitive(
            mode, (NfPrimitiveId)i);
        const NfCurrentPrimitive *baseline = &profile->current[i];
        const char *primitive = nf_slalom_primitive_geometry[i].name;

        nf_check(actual.available == baseline->available, profile->name,
                 primitive, "availability differs from compiled parameters");
        nf_check_close(baseline->velocity_mm_s, actual.velocity_mm_s, 1.0e-5,
                       profile->name, primitive,
                       "velocity differs from compiled parameters");
        nf_check_close(baseline->alpha_deg_s2, actual.alpha_deg_s2, 1.0e-5,
                       profile->name, primitive,
                       "alpha differs from compiled parameters");
        nf_check_close(baseline->dist_in_mm, actual.dist_in_mm, 1.0e-5,
                       profile->name, primitive,
                       "dist_in differs from compiled parameters");
        nf_check_close(baseline->dist_out_mm, actual.dist_out_mm, 1.0e-5,
                       profile->name, primitive,
                       "dist_out differs from compiled parameters");
    }
}

static void nf_audit_seed(const NfAuditProfile *profile,
                          const NfPrimitiveGeometry *geometry,
                          const NfCurrentPrimitive *current,
                          const NfProvisionalSeed *seed)
{
    const NfTurnSpec turn = nf_make_turn(
        geometry->angle_deg, seed->velocity_mm_s, seed->alpha_deg_s2,
        seed->dist_in_mm, seed->dist_out_mm);
    NfTurnPlan plan = {0};
    const NfMotionStatus status =
        nf_motion_turn_plan(&turn, &profile->environment, &plan);
    double residual_forward_mm = INFINITY;
    double residual_lateral_mm = INFINITY;
    double residual_norm_mm = INFINITY;

    nf_check(seed->velocity_basis != NULL, profile->name, geometry->name,
             "provisional seed is missing its velocity basis");
    nf_check(status == NF_MOTION_OK, profile->name, geometry->name,
             "provisional seed did not produce a turn plan");
    if (status == NF_MOTION_OK) {
        residual_forward_mm =
            plan.displacement_forward_mm - geometry->target_forward_mm;
        residual_lateral_mm =
            plan.displacement_lateral_mm - geometry->target_lateral_mm;
        residual_norm_mm = hypot(residual_forward_mm, residual_lateral_mm);
        nf_check(residual_norm_mm <= NF_SEED_CLOSURE_TOLERANCE_MM,
                 profile->name, geometry->name,
                 "provisional seed exceeds the 0.001 mm closure tolerance");
        nf_check_mirror(profile->name, geometry, &plan,
                        NF_MIRROR_TOLERANCE_MM);
    }
    if (current->available) {
        nf_check_close(seed->velocity_mm_s, current->velocity_mm_s, 0.0,
                       profile->name, geometry->name,
                       "provisional seed changed center velocity");
    }

    printf("  seed=PROVISIONAL_NOT_FIRMWARE velocity=%.3f alpha=%.3f "
           "dist_in=%.3f dist_out=%.3f basis=%s closure=(%+.6f,%+.6f) "
           "norm=%.6f\n",
           seed->velocity_mm_s, seed->alpha_deg_s2,
           seed->dist_in_mm, seed->dist_out_mm, seed->velocity_basis,
           residual_forward_mm, residual_lateral_mm, residual_norm_mm);
}

static void nf_audit_profile(size_t profile_index,
                             const NfAuditProfile *profile)
{
    printf("profile=%s role=%s source=%s omega_cap=%.1f rounding_scale=%.1f\n",
           profile->name, profile->primary ? "primary" : "comparison",
           profile->source, profile->environment.omega_cap_deg_s,
           profile->environment.rounding_scale);
    nf_audit_compiled_source(profile_index, profile);

    for (size_t i = 0U; i < NF_PRIMITIVE_COUNT; i++) {
        const NfPrimitiveGeometry *geometry =
            &nf_slalom_primitive_geometry[i];
        const NfCurrentPrimitive *current = &profile->current[i];
        const NfProvisionalSeed *seed = &profile->seeds[i];
        NfTurnPlan current_plan = {0};
        const bool timing_is_provisional =
            i >= (size_t)NF_PRIMITIVE_45_IN;

        if (!current->available) {
            printf(" primitive=%s target=(%.6f,%.6f) current=UNAVAILABLE "
                   "reason=profile-fields-zero\n",
                   geometry->name, geometry->target_forward_mm,
                   geometry->target_lateral_mm);
        } else {
            const NfTurnSpec turn = nf_make_turn(
                geometry->angle_deg, current->velocity_mm_s,
                current->alpha_deg_s2, current->dist_in_mm,
                current->dist_out_mm);
            const NfMotionStatus status =
                nf_motion_turn_plan(&turn, &profile->environment, &current_plan);
            const bool plan_valid = status == NF_MOTION_OK;
            double current_forward_mm = NAN;
            double current_lateral_mm = NAN;
            double residual_forward_mm = INFINITY;
            double residual_lateral_mm = INFINITY;
            double residual_norm_mm = INFINITY;

            nf_check(plan_valid, profile->name, geometry->name,
                     "current parameters did not produce a turn plan");
            if (plan_valid) {
                current_forward_mm = current_plan.displacement_forward_mm;
                current_lateral_mm = current_plan.displacement_lateral_mm;
                residual_forward_mm = current_forward_mm -
                                      geometry->target_forward_mm;
                residual_lateral_mm = current_lateral_mm -
                                      geometry->target_lateral_mm;
                residual_norm_mm = hypot(residual_forward_mm,
                                         residual_lateral_mm);
                nf_check_close(current_forward_mm,
                               current->expected_forward_mm,
                               NF_BASELINE_REGRESSION_TOLERANCE_MM,
                               profile->name, geometry->name,
                               "current forward baseline changed");
                nf_check_close(current_lateral_mm,
                               current->expected_lateral_mm,
                               NF_BASELINE_REGRESSION_TOLERANCE_MM,
                               profile->name, geometry->name,
                               "current lateral baseline changed");
                nf_check_mirror(profile->name, geometry, &current_plan,
                                NF_MIRROR_TOLERANCE_MM);
            }
            printf(" primitive=%s target=(%.6f,%.6f) current=(%.6f,%.6f) "
                   "residual=(%+.6f,%+.6f) norm=%.6f status=%s\n",
                   geometry->name, geometry->target_forward_mm,
                   geometry->target_lateral_mm,
                   current_forward_mm, current_lateral_mm,
                   residual_forward_mm, residual_lateral_mm,
                   residual_norm_mm,
                   timing_is_provisional
                       ? "PROVISIONAL_DIAGONAL_TIMING"
                       : "CURRENT_CALIBRATED_ORTHOGONAL_TIMING");
        }

        nf_check(seed->available,
                 profile->name, geometry->name,
                 "every primitive needs an exact-closing PC geometry seed");
        if (seed->available) {
            nf_audit_seed(profile, geometry, current, seed);
        }
    }
}

int main(void)
{
    const double computed_logical_diagonal_mm =
        NF_SLALOM_HALF_CELL_MM * sqrt(2.0);
    const double command_delta_mm =
        NF_SLALOM_COMMAND_DIAGONAL_MM - NF_SLALOM_LOGICAL_DIAGONAL_MM;

    nf_check_close(NF_SLALOM_LOGICAL_DIAGONAL_MM,
                   computed_logical_diagonal_mm,
                   1.0e-12, "global", "diagonal-distance",
                   "logical diagonal constant is not 45*sqrt(2)");
    nf_check(command_delta_mm > 0.0, "global", "diagonal-distance",
             "command diagonal distance must remain separately reported");
    printf("diagonal-distance logical_45sqrt2_mm=%.12f "
           "command_DIST_D_HALF_SEC_mm=%.12f delta_mm=%+.12f "
           "delta_percent=%.6f\n",
           NF_SLALOM_LOGICAL_DIAGONAL_MM, NF_SLALOM_COMMAND_DIAGONAL_MM,
           command_delta_mm,
           100.0 * command_delta_mm / NF_SLALOM_LOGICAL_DIAGONAL_MM);
    printf("notice=all-seeds-are-PC-only-provisional;firmware-parameters-unchanged\n");

    for (size_t i = 0U; i < nf_slalom_profile_count; i++) {
        nf_audit_profile(i, &nf_slalom_profiles[i]);
    }

    if (g_failures != 0U) {
        fprintf(stderr, "slalom_profile_audit: %u/%u checks failed\n",
                g_failures, g_checks);
        return 1;
    }
    printf("slalom_profile_audit: all %u checks passed\n", g_checks);
    return 0;
}
