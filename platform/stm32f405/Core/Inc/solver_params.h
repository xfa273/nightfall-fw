#ifndef INC_SOLVER_PARAMS_H_
#define INC_SOLVER_PARAMS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 新しいソルバ（solver.c）のコストパラメータ
typedef struct {
    float move_cost_normal;     // 通常移動
    float move_cost_straight;   // 直線継続
    float move_cost_diagonal;   // 斜め継続パターン
    float move_cost_min;        // 最小コスト
    float straight_discount;    // 直線割引/step
    float diagonal_discount;    // 斜め割引/step
    float turn_penalty;         // 方向転換ペナルティ
} SolverCaseParams_t;

// プロファイルID
typedef enum {
    SOLVER_PROFILE_STANDARD = 0,      // 標準
    SOLVER_PROFILE_STRAIGHT_STRONG,   // 直進をより強く優先
    SOLVER_PROFILE_STRAIGHT_WEAK,     // 直進優先を弱め
} solver_profile_t;

// 現在のプロファイルを設定/取得
void solver_set_profile(uint8_t profile);
uint8_t solver_get_profile(void);

// パラメータを取得（現在のプロファイルに基づく）
const SolverCaseParams_t* solver_get_case_params(uint8_t mode, uint8_t case_index);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOLVER_PARAMS_H_ */
