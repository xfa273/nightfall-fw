#include "solver_params.h"
#include <stddef.h>

// move_cost_min は安全のため固定値
#define SOLVER_MOVE_COST_MIN 0.05f

// =============================================
// 3つのプロファイル定義
// =============================================

// 標準プロファイル
static const SolverCaseParams_t kProfileStandard = {
    .move_cost_normal   = 1.00f,
    .move_cost_straight = 0.80f,
    .move_cost_diagonal = 0.85f,
    .move_cost_min      = SOLVER_MOVE_COST_MIN,
    .straight_discount  = 0.01f,
    .diagonal_discount  = 0.005f,
    .turn_penalty       = 0.80f,
};

// 直進をより強く優先（直進コストを下げ、ターンペナルティを増やす）
static const SolverCaseParams_t kProfileStraightStrong = {
    .move_cost_normal   = 1.00f,
    .move_cost_straight = 0.50f,
    .move_cost_diagonal = 1.00f,
    .move_cost_min      = SOLVER_MOVE_COST_MIN,
    .straight_discount  = 0.02f,
    .diagonal_discount  = 0.00f,
    .turn_penalty       = 1.50f,
};

// 直進優先を弱める（それでも多少は直進を優先）
static const SolverCaseParams_t kProfileStraightWeak = {
    .move_cost_normal   = 1.00f,
    .move_cost_straight = 0.90f,
    .move_cost_diagonal = 0.80f,
    .move_cost_min      = SOLVER_MOVE_COST_MIN,
    .straight_discount  = 0.005f,
    .diagonal_discount  = 0.01f,
    .turn_penalty       = 0.50f,
};

// =============================================
// プロファイル管理
// =============================================

static uint8_t g_profile = SOLVER_PROFILE_STANDARD;
static SolverCaseParams_t g_params;

void solver_set_profile(uint8_t profile) {
    switch (profile) {
        case SOLVER_PROFILE_STANDARD:
        case SOLVER_PROFILE_STRAIGHT_STRONG:
        case SOLVER_PROFILE_STRAIGHT_WEAK:
            g_profile = profile;
            break;
        default:
            g_profile = SOLVER_PROFILE_STANDARD;
            break;
    }
}

uint8_t solver_get_profile(void) {
    return g_profile;
}

const SolverCaseParams_t* solver_get_case_params(uint8_t mode, uint8_t case_index) {
    (void)mode;
    (void)case_index;
    
    // 現在のプロファイルに応じてパラメータを返却
    switch (g_profile) {
        case SOLVER_PROFILE_STRAIGHT_STRONG:
            g_params = kProfileStraightStrong;
            break;
        case SOLVER_PROFILE_STRAIGHT_WEAK:
            g_params = kProfileStraightWeak;
            break;
        case SOLVER_PROFILE_STANDARD:
        default:
            g_params = kProfileStandard;
            break;
    }
    
    // 安全のため move_cost_min を固定値へ上書き
    g_params.move_cost_min = SOLVER_MOVE_COST_MIN;
    return &g_params;
}
