/*
 * path.c
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#include "global.h"

void simplifyPath(void) {
    static uint16_t simplifiedPath[ROUTE_MAX_LEN]; // 結果を格納するための配列
    int currentIndex = 0; // simplifiedPathにおける現在のインデックス
    int currentAction = path[0]; // 現在処理している動作
    int count = 1;               // 現在の動作のカウント
    int i = 1;

    while (path[i] != 0) {
        if (path[i] == currentAction && currentAction == STRAIGHT) {
            // 現在の動作が直進で続いている場合、カウントを増やす
            count++;
        } else {
            // 直進以外の場合、または動作が変わった場合
            if (currentAction == STRAIGHT) {
                // 直進の場合、カウントを加えて結果配列に追加
                simplifiedPath[currentIndex++] = currentAction + count * 2;
            } else {
                // 直進以外の場合はそのまま結果配列に追加
                simplifiedPath[currentIndex++] = currentAction;
            }

            // 新しい動作の処理を開始
            currentAction = path[i];
            count = 1;
        }
        i++;
    }

    // 最後の動作を結果配列に追加
    if (currentAction == STRAIGHT) {
        simplifiedPath[currentIndex++] = currentAction + count * 2;
    } else {
        simplifiedPath[currentIndex++] = currentAction;
    }

    // simplifiedPathをpathにコピー
    for (int j = 0; j < currentIndex; j++) {
        path[j] = simplifiedPath[j];
    }

    // pathの残りをクリア
    for (int j = currentIndex; j < ROUTE_MAX_LEN; j++) {
        path[j] = 0;
    }

    path[0] -= 1;
}

void convertLTurn() {
    static uint16_t convertedPath[ROUTE_MAX_LEN];
    int i = 0;
    int j = 0;

    while (path[i] != 0) {

        if (path[i] == 300 && path[i + 1] == 300) {
            // 右旋回が2連続

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 2] > 200 &&
                path[i + 2] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 502;
                path[i + 2] -= 1;

                i++;
            } else {
                // 通常右旋回
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 400 && path[i + 1] == 400) {
            // 左旋回が2連続

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 2] > 200 &&
                path[i + 2] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 602;
                path[i + 2] -= 1;

                i++;
            } else {
                // 通常左旋回
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 300) {
            // 右旋回

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 1] > 200 &&
                path[i + 1] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 501;
                path[i + 1] -= 1;
            } else {
                // 通常右旋回
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 400) {
            // 左旋回

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 1] > 200 &&
                path[i + 1] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 601;
                path[i + 1] -= 1;
            } else {
                // 通常左旋回
                convertedPath[j] = path[i];
            }
        } else {
            // 直進
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < ROUTE_MAX_LEN; l++) {
        path[l] = 0;
    }

    int count = 0;
    int m, n;

    // 値が200の要素を削除し、後ろの要素を前に詰める
    for (m = 0; m < ROUTE_MAX_LEN - count; m++) {
        if (path[m] == 200) {
            for (n = m; n < (ROUTE_MAX_LEN - 1) - count; n++) {
                path[n] = path[n + 1];
            }
            count++; // 削除した要素の数をインクリメント
            m--; // 詰めた後の現在の位置にある新しい要素もチェックするためにデクリメント
        }
    }
}

void convertDiagonal(void) {
    static uint16_t convertedPath[ROUTE_MAX_LEN];
    int i = 0;
    int j = 0;

    /* 小回りの開始の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 400 && path[i - 1] < 300) {
            // 右小回りの開始

            if (path[i + 1] >= 300 && path[i + 1] < 400) {
                // 右小回りx2

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 901; // 右斜め135°に変換
                } else {
                    // 直前がS1の場合は削除せず保持し、続きに135°入りを挿入
                    convertedPath[j] = 901; // 右斜め135°に変換（直前の直進は保持）
                }

                i++; // 次の小回りとまとめたので次パスをスキップ
            } else {
                // 右小回りx1

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 701;     // 右斜め45°に変換
                } else {
                    // 直前がS1の場合は削除せず保持し、その後に45°入りを挿入
                    convertedPath[j] = 701;     // 右斜め45°に変換（S1は保持）
                }

                // j--
            }
        } else if (path[i] >= 400 && path[i] < 500 && path[i - 1] < 300) {
            // 左小回りの開始

            if (path[i + 1] >= 400 && path[i + 1] < 500) {
                // 左小回りx2

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 902; // 左斜め135°に変換
                } else {
                    // 直前がS1の場合は削除せず保持し、続きに135°入りを挿入
                    convertedPath[j] = 902; // 左斜め135°に変換（直前の直進は保持）
                }
                i++; // 次の小回りとまとめたので次パスをスキップ
            } else {
                // 左小回りx1

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 702; // 左斜め45°に変換
                } else {
                    // 直前がS1の場合は削除せず保持し、その後に45°入りを挿入
                    convertedPath[j] = 702; // 左斜め45°に変換（直前の直進は保持）
                }
                // j--
            }
        } else {
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < ROUTE_MAX_LEN; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < ROUTE_MAX_LEN && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    /* 小回りの終了の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 400 &&
            (path[i + 1] < 300 || path[i + 1] > 500)) {
            // 右小回りの終了

            if (path[i - 1] >= 300 && path[i - 1] < 400) {
                // 右小回りx2

                convertedPath[j - 1] = 903; // 右斜め135°に変換
                // 前の小回りとまとめたので1つ前のパスを上書き
                j--;

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後がS1の場合は削除せず保持
                    convertedPath[j + 1] = 201; // S1 を保持
                    j++;
                }
                i++;

            } else {
                // 右小回りx1

                convertedPath[j] = 703; // 右斜め45°に変換

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後がS1の場合
                    // パターンが [703,201,701/702] のときは S1 を挟まず即座に斜め入りを出力して連結
                    if (path[i + 2] == 701 || path[i + 2] == 702) {
                        convertedPath[j + 1] = path[i + 2]; // 703 の直後に 701/702 を出力
                        j += 2;   // 703 と 701/702 の2要素を書いた
                        i += 3;   // 入力側から 703, S1, 701/702 を消費
                        continue; // 次の入力要素から処理を再開
                    } else {
                        // それ以外はS1を保持
                        convertedPath[j + 1] = 201; // S1 を保持
                        j++;
                    }
                }
                i++;
            }
        } else if (path[i] >= 400 && path[i] < 500 &&
                   (path[i + 1] < 300 || path[i + 1] > 500)) {
            // 左小回りの終了

            if (path[i - 1] >= 400 && path[i - 1] < 500) {
                // 左小回りx2

                convertedPath[j - 1] = 904; // 左斜め135°に変換
                // 前の小回りとまとめたので1つ前のパスを上書き
                j--;

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後がS1の場合は削除せず保持
                    convertedPath[j + 1] = 201; // S1 を保持
                    j++;
                }
                i++;
            } else {
                // 左小回りx1

                convertedPath[j] = 704; // 左斜め45°に変換

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後がS1の場合
                    // パターンが [704,201,701/702] のときは S1 を挟まず即座に斜め入りを出力して連結
                    if (path[i + 2] == 701 || path[i + 2] == 702) {
                        convertedPath[j + 1] = path[i + 2]; // 704 の直後に 701/702 を出力
                        j += 2;   // 704 と 701/702 の2要素を書いた
                        i += 3;   // 入力側から 704, S1, 701/702 を消費
                        continue; // 次の入力要素から処理を再開
                    } else {
                        // それ以外はS1を保持
                        convertedPath[j + 1] = 201; // S1 を保持
                        j++;
                    }
                }
                i++;
            }
        } else {
            // 非該当（直進など）はそのままコピー
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < ROUTE_MAX_LEN; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < ROUTE_MAX_LEN && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    /* V90の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 400 && path[i + 1] >= 300 &&
            path[i + 1] < 400) {
            // 右小回りの連続

            convertedPath[j] = 801; // 右V90に変換
            i++; // 次の小回りとまとめたので次パスをスキップ

        } else if (path[i] >= 400 && path[i] < 500 && path[i + 1] >= 400 &&
                   path[i + 1] < 500) {
            // 左小回りの連続

            convertedPath[j] = 802; // 左V90に変換
            i++; // 次の小回りとまとめたので次パスをスキップ

        } else {
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < ROUTE_MAX_LEN; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < ROUTE_MAX_LEN && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    /* 斜め直進の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 500) {
            // 小回り

            convertedPath[j] = 1001; // 斜め直進に変換

        } else {
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < ROUTE_MAX_LEN; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < ROUTE_MAX_LEN && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    int result_index = 0; // convertedPathのインデックス
    int sum = 0;          // 連続した1000以上の要素の積算値
    int in_sequence = 0;  // 1000以上の連続のフラグ

    /* 斜め直進を繋いでまとめる処理 */
    while (path[i] != 0) {
        if (path[i] >= 1000) {
            // 1000以上の連続が始まった場合
            if (!in_sequence) {
                in_sequence = 1;
                sum = 0; // 積算値をリセット
            }
            sum += path[i] - 1000; // 1000を引いた値を積算
        } else {
            // 1000未満の要素に達した場合
            if (in_sequence) {
                // 連続が終わったときに積算値をconvertedPathに格納
                convertedPath[result_index++] = sum + 1000;
                j++;
                in_sequence = 0;
            }
            convertedPath[result_index++] = path[i];
            j++;
        }
        i++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < ROUTE_MAX_LEN; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < ROUTE_MAX_LEN; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < ROUTE_MAX_LEN && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");
}

// makePath()は削除済み - 経路導出はsolver_build_path()を使用
