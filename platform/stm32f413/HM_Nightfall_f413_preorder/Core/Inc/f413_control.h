/**
 * @file f413_control.h
 * @brief F413 最小限クローズドループ速度/角速度制御
 *
 * TIM5 1kHz 割り込みでエンコーダを読み取り、P制御で
 * TIM2 (モータPWM) を駆動する。
 */

#ifndef F413_CONTROL_H_
#define F413_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初期化（TIM5割り込み開始） — main初期化後に1回呼ぶ */
void f413_ctrl_init(void);

/* 制御開始/停止 */
void f413_ctrl_start(void);
void f413_ctrl_stop(void);

/* 目標値設定（走行中に随時呼ぶ） */
void f413_ctrl_set_velocity(float velocity_mm_s);
void f413_ctrl_set_omega(float omega_deg_s);

/* 累積量取得 */
float f413_ctrl_get_distance(void);
float f413_ctrl_get_angle(void);

/* 累積量リセット */
void f413_ctrl_reset_distance(void);
void f413_ctrl_reset_angle(void);

/* 実測値取得（デバッグ/ログ用） */
float f413_ctrl_get_real_velocity(void);
float f413_ctrl_get_real_omega(void);

/* 1kHz 割り込みハンドラ（HAL_TIM_PeriodElapsedCallback から呼ぶ） */
void f413_ctrl_tick(void);

/* 制御が有効かどうか */
bool f413_ctrl_is_running(void);

/* SPI2 バス排他: 割り込みコンテキストの IMU 読取中は true を返す。
   メインループ側の FRAM 操作はこれが false の間のみ安全。 */
bool f413_ctrl_spi2_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* F413_CONTROL_H_ */
