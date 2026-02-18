#ifndef INC_SOLVER_H_
#define INC_SOLVER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// モード（2..7）とケース（3..7）を指定して新ソルバを実行し、
// 結果を ASCII 迷路にオーバレイ（path_cell）して printMaze() で出力します。
void solver_run(uint8_t mode, uint8_t case_index);

// 最短走行で使用する経路を生成（path[] と path_cell[] を設定）
// - map->maze の構築を内部で行います
// - 生成後は simplifyPath(), convertLTurn(), convertDiagonal() など従来の後段処理を利用可能
// - 経路生成成功時 true、失敗時 false を返す
bool solver_build_path(uint8_t mode, uint8_t case_index);

#ifdef __cplusplus
}
#endif

#endif /* INC_SOLVER_H_ */
