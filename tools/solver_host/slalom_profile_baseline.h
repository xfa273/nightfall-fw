#ifndef NIGHTFALL_TOOLS_SOLVER_HOST_SLALOM_PROFILE_BASELINE_H
#define NIGHTFALL_TOOLS_SOLVER_HOST_SLALOM_PROFILE_BASELINE_H

#include "motion_time.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NF_SLALOM_HALF_CELL_MM 45.0
#define NF_SLALOM_LOGICAL_DIAGONAL_MM 63.639610306789280
#define NF_SLALOM_COMMAND_DIAGONAL_MM 67.279

typedef enum {
    NF_PRIMITIVE_SMALL_90 = 0,
    NF_PRIMITIVE_LARGE_90,
    NF_PRIMITIVE_LARGE_180,
    NF_PRIMITIVE_45_IN,
    NF_PRIMITIVE_45_OUT,
    NF_PRIMITIVE_V90,
    NF_PRIMITIVE_135_IN,
    NF_PRIMITIVE_135_OUT,
    NF_PRIMITIVE_COUNT,
} NfPrimitiveId;

typedef struct {
    const char *name;
    double angle_deg;
    double target_forward_mm;
    double target_lateral_mm;
} NfPrimitiveGeometry;

typedef struct {
    bool available;
    double velocity_mm_s;
    double alpha_deg_s2;
    double dist_in_mm;
    double dist_out_mm;
    double expected_forward_mm;
    double expected_lateral_mm;
} NfCurrentPrimitive;

typedef struct {
    bool available;
    double velocity_mm_s;
    double alpha_deg_s2;
    double dist_in_mm;
    double dist_out_mm;
    const char *velocity_basis;
} NfProvisionalSeed;

typedef struct {
    const char *name;
    const char *source;
    bool primary;
    uint8_t shortest_run_mode;
    double start_offset_mm;
    NfTurnEnvironment environment;
    const NfCurrentPrimitive *current;
    const NfProvisionalSeed *seeds;
} NfAuditProfile;

extern const NfPrimitiveGeometry
    nf_slalom_primitive_geometry[NF_PRIMITIVE_COUNT];
extern const NfAuditProfile nf_slalom_profiles[];
extern const size_t nf_slalom_profile_count;

const NfAuditProfile *nf_slalom_profile_find(const char *name);

#ifdef __cplusplus
}
#endif

#endif
