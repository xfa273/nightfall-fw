#include "slalom_profile_baseline.h"

#include <string.h>

#define NF_ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))
#define NF_CURRENT(v, a, in, out, x, y) \
    { true, (v), (a), (in), (out), (x), (y) }
#define NF_CURRENT_UNAVAILABLE \
    { false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
#define NF_SEED(v, a, in, out, basis) \
    { true, (v), (a), (in), (out), (basis) }

const NfPrimitiveGeometry
    nf_slalom_primitive_geometry[NF_PRIMITIVE_COUNT] = {
    [NF_PRIMITIVE_SMALL_90] = {
        "small90", 90.0, NF_SLALOM_HALF_CELL_MM, NF_SLALOM_HALF_CELL_MM,
    },
    [NF_PRIMITIVE_LARGE_90] = {
        "large90", 90.0,
        2.0 * NF_SLALOM_HALF_CELL_MM, 2.0 * NF_SLALOM_HALF_CELL_MM,
    },
    [NF_PRIMITIVE_LARGE_180] = {
        "large180", 180.0, 0.0, 2.0 * NF_SLALOM_HALF_CELL_MM,
    },
    [NF_PRIMITIVE_45_IN] = {
        "45in", 45.0, 2.0 * NF_SLALOM_HALF_CELL_MM, NF_SLALOM_HALF_CELL_MM,
    },
    [NF_PRIMITIVE_45_OUT] = {
        "45out", 45.0, 95.459415460183920, 31.819805153394640,
    },
    [NF_PRIMITIVE_V90] = {
        "v90", 90.0,
        NF_SLALOM_LOGICAL_DIAGONAL_MM, NF_SLALOM_LOGICAL_DIAGONAL_MM,
    },
    [NF_PRIMITIVE_135_IN] = {
        "135in", 135.0,
        NF_SLALOM_HALF_CELL_MM, 2.0 * NF_SLALOM_HALF_CELL_MM,
    },
    [NF_PRIMITIVE_135_OUT] = {
        "135out", 135.0, 31.819805153394640, 95.459415460183920,
    },
};

/* F413 preorder mode 2 after overhead-video turn calibration. */
static const NfCurrentPrimitive g_current_mode2[NF_PRIMITIVE_COUNT] = {
    NF_CURRENT(300.0, 10200.0, 7.2, 11.1,
               43.712055, 47.612055),
    NF_CURRENT(500.0, 6900.0, 13.0, 20.5,
               86.987877, 94.487877),
    NF_CURRENT(500.0, 4150.0, 6.5, 20.0,
               -13.500000, 92.904831),
    NF_CURRENT(500.0, 12000.0, 10.0, 29.0,
               90.152412, 45.212409),
    NF_CURRENT(500.0, 10500.0, 7.5, 0.0,
               71.264593, 26.412159),
    NF_CURRENT(500.0, 12200.0, 12.8, 27.2,
               68.442316, 82.842316),
    NF_CURRENT(500.0, 8400.0, 18.0, 10.5,
               44.772552, 89.983901),
    NF_CURRENT(500.0, 8850.0, 0.0, 12.0,
               24.831130, 88.918214),
};

static const NfCurrentPrimitive g_current_f405_mode2[NF_PRIMITIVE_COUNT] = {
    NF_CURRENT(300.0, 8920.0, 10.0, 14.2,
               49.043967006716, 53.243967006716),
    NF_CURRENT(500.0, 4700.0, 5.0, 15.0,
               94.647087582099, 104.647087582099),
    NF_CURRENT(500.0, 4697.0, 12.0, 19.0,
               -7.000000000000, 87.327708585673),
    NF_CURRENT(500.0, 6360.0, 0.0, 28.0,
               101.729504843293, 53.735720346038),
    NF_CURRENT(500.0, 7700.0, 15.0, 0.0,
               89.461002642226, 30.842757162309),
    NF_CURRENT(500.0, 12200.0, 2.0, 28.0,
               57.642316030996, 83.642316030996),
    NF_CURRENT(500.0, 6888.0, 9.0, 17.0,
               34.743645346995, 103.192288301982),
    NF_CURRENT(500.0, 6950.0, 0.0, 12.0,
               29.110356101792, 99.249179254933),
};

static const NfCurrentPrimitive g_current_mode3[NF_PRIMITIVE_COUNT] = {
    NF_CURRENT(600.0, 36000.0, 4.0, 14.0,
               42.870050921591, 52.870050921591),
    NF_CURRENT(1000.0, 17300.0, 1.0, 6.0,
               94.452744036237, 99.452744036236),
    NF_CURRENT(1000.0, 15000.0, 0.0, 0.0,
               0.0, 97.734233431980),
    NF_CURRENT(1000.0, 27200.0, 0.0, 35.0,
               103.984234290163, 57.569154799025),
    NF_CURRENT(1000.0, 28000.0, 17.0, 0.0,
               95.095358414788, 32.348156613793),
    NF_CURRENT(1000.0, 43000.0, 6.0, 23.0,
               65.276271942982, 82.276271942982),
    NF_CURRENT(1000.0, 26500.0, 5.0, 25.0,
               25.829084796243, 110.641198066238),
    NF_CURRENT(1000.0, 29000.0, 0.0, 26.0,
               18.424802844848, 107.250961533783),
};

static const NfCurrentPrimitive g_current_mode4[NF_PRIMITIVE_COUNT] = {
    NF_CURRENT(800.0, 51500.0, 4.0, 14.0,
               47.331276390830, 57.331276390830),
    NF_CURRENT(1000.0, 17300.0, 1.0, 6.0,
               94.452744036237, 99.452744036236),
    NF_CURRENT(1000.0, 15000.0, 0.0, 0.0,
               0.0, 97.734233431980),
    NF_CURRENT(1200.0, 16422.0, 0.0, 20.0,
               136.511431265443, 64.829157496571),
    NF_CURRENT(1200.0, 16422.0, 18.0, 2.0,
               141.783509204085, 52.101235435213),
    NF_CURRENT(1200.0, 25838.0, 5.0, 7.0,
               96.762853193989, 98.762853193989),
    NF_CURRENT(1200.0, 17395.0, 8.0, 2.0,
               63.619136293580, 139.104900292186),
    NF_CURRENT(1200.0, 17736.0, 2.0, 10.0,
               51.411347148970, 143.431680045899),
};

static const NfCurrentPrimitive g_current_mode5[NF_PRIMITIVE_COUNT] = {
    NF_CURRENT(800.0, 51500.0, 4.0, 14.0,
               47.331276390830, 57.331276390830),
    NF_CURRENT(1200.0, 24800.0, 2.0, 8.0,
               95.663528646939, 101.663528646940),
    NF_CURRENT(1200.0, 21000.0, 0.0, 4.0,
               -4.000000000000, 99.120603861202),
    NF_CURRENT_UNAVAILABLE,
    NF_CURRENT_UNAVAILABLE,
    NF_CURRENT_UNAVAILABLE,
    NF_CURRENT_UNAVAILABLE,
    NF_CURRENT_UNAVAILABLE,
};

static const NfProvisionalSeed g_seed_mode2[NF_PRIMITIVE_COUNT] = {
    NF_SEED(300.0, 8920.0, 5.956032993, 5.956032993,
            "current-turn-velocity-exact-closure"),
    NF_SEED(500.0, 4700.0, 0.352912418, 0.352912418,
            "current-turn-velocity-exact-closure"),
    NF_SEED(500.0, 4422.213141, 15.500, 15.500,
            "current-turn-velocity-exact-closure"),
    NF_SEED(500.0, 7234.4, 0.0, 18.640, "current-turn-velocity"),
    NF_SEED(500.0, 7234.4, 18.640, 0.0, "current-turn-velocity"),
    NF_SEED(500.0, 12200.0, 7.997, 7.997, "current-turn-velocity"),
    NF_SEED(500.0, 8500.0, 18.932, 11.212, "current-turn-velocity"),
    NF_SEED(500.0, 8500.0, 11.212, 18.932, "current-turn-velocity"),
};

static const NfProvisionalSeed g_seed_mode3[NF_PRIMITIVE_COUNT] = {
    NF_SEED(600.0, 36000.0, 6.129949078, 6.129949078,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1000.0, 20000.0, 3.083923850, 3.083923850,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1000.0, 17688.852564, 0.0, 0.0,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1000.0, 28937.6, 0.0, 18.640, "current-turn-velocity"),
    NF_SEED(1000.0, 28937.6, 18.640, 0.0, "current-turn-velocity"),
    NF_SEED(1000.0, 43000.0, 4.363, 4.363, "current-turn-velocity"),
    NF_SEED(1000.0, 34000.0, 18.932, 11.212, "current-turn-velocity"),
    NF_SEED(1000.0, 34000.0, 11.212, 18.932, "current-turn-velocity"),
};

static const NfProvisionalSeed g_seed_mode4[NF_PRIMITIVE_COUNT] = {
    NF_SEED(800.0, 51500.0, 1.668723609, 1.668723609,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1000.0, 20000.0, 3.083923850, 3.083923850,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1000.0, 17688.852564, 0.0, 0.0,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1200.0, 41670.1, 0.0, 18.640, "current-turn-velocity"),
    NF_SEED(1200.0, 41670.1, 18.640, 0.0, "current-turn-velocity"),
    NF_SEED(1200.0, 53720.2, 0.0, 0.0, "current-turn-velocity"),
    NF_SEED(1200.0, 48960.0, 18.932, 11.212, "current-turn-velocity"),
    NF_SEED(1200.0, 48960.0, 11.212, 18.932, "current-turn-velocity"),
};

static const NfProvisionalSeed g_seed_mode5[NF_PRIMITIVE_COUNT] = {
    NF_SEED(800.0, 51500.0, 1.668723609, 1.668723609,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1200.0, 30000.0, 4.839985195, 4.839985195,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1200.0, 25471.947692, 2.0, 2.0,
            "current-turn-velocity-exact-closure"),
    NF_SEED(1200.0, 41670.1, 0.0, 18.640,
            "assumed-mode5-large-turn-velocity"),
    NF_SEED(1200.0, 41670.1, 18.640, 0.0,
            "assumed-mode5-large-turn-velocity"),
    NF_SEED(1200.0, 53720.2, 0.0, 0.0,
            "assumed-mode5-large-turn-velocity"),
    NF_SEED(1200.0, 48960.0, 18.932, 11.212,
            "assumed-mode5-large-turn-velocity"),
    NF_SEED(1200.0, 48960.0, 11.212, 18.932,
            "assumed-mode5-large-turn-velocity"),
};

const NfAuditProfile nf_slalom_profiles[] = {
    {
        "f413-preorder-mode2",
        "params/f413_preorder/shortest_run_params_split.c",
        true,
        2U,
        5.0,
        {2200.0, 1.2},
        g_current_mode2,
        g_seed_mode2,
    },
    {
        "f405-mini-mode2",
        "params/mini_r1_0/shortest_run_params_split.c",
        false,
        2U,
        13.0,
        {0.0, 1.2},
        g_current_f405_mode2,
        g_seed_mode2,
    },
    {
        "f405-mini-mode3",
        "params/mini_r1_0/shortest_run_params_split.c",
        false,
        3U,
        13.0,
        {0.0, 1.2},
        g_current_mode3,
        g_seed_mode3,
    },
    {
        "f405-mini-mode4",
        "params/mini_r1_0/shortest_run_params_split.c",
        false,
        4U,
        13.0,
        {0.0, 1.2},
        g_current_mode4,
        g_seed_mode4,
    },
    {
        "f405-mini-mode5",
        "params/mini_r1_0/shortest_run_params_split.c",
        false,
        5U,
        13.0,
        {0.0, 1.2},
        g_current_mode5,
        g_seed_mode5,
    },
};

const size_t nf_slalom_profile_count = NF_ARRAY_COUNT(nf_slalom_profiles);

const NfAuditProfile *nf_slalom_profile_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0U; i < nf_slalom_profile_count; i++) {
        if (strcmp(nf_slalom_profiles[i].name, name) == 0) {
            return &nf_slalom_profiles[i];
        }
    }
    return NULL;
}
