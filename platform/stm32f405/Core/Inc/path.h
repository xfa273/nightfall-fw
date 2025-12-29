/*
 * path.h
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#ifndef INC_PATH_H_
#define INC_PATH_H_

// 走行パスコード
#define STRAIGHT 200
#define TURN_R 300
#define TURN_L 400
#define L_TURN_R 500
#define L_TURN_L 600

void simplifyPath(void);
void convertLTurn(void);
void convertDiagonal(void);
// makePath()は削除済み - 経路導出はsolver_build_path()を使用

#endif /* INC_PATH_H_ */

/*
200 直進
300 右小回り
400 左小回り
500 右大回り
600 左大回り
700 45°ターン 701:右入り 702:左入り 703:右出 704:左出
800 V90ターン 801:右 802:左
900 135°ターン 901:右入り 902:左入り 903:右出 904:左出
1000 斜め直進
*/
