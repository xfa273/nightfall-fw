/*
 * control.h
 *
 *  Created on: 2023/07/29
 *      Author: yuho-
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

/*============================================================
    関数プロトタイプ宣言
============================================================*/
#ifndef CTRL_ENABLE_ANTI_WINDUP
#define CTRL_ENABLE_ANTI_WINDUP 0
#endif

#ifndef CTRL_OUTPUT_MAX
#define CTRL_OUTPUT_MAX 1000.0f
#endif

#ifndef CTRL_DISTANCE_OUTER_DIV
#define CTRL_DISTANCE_OUTER_DIV 1
#endif

#if (CTRL_DISTANCE_OUTER_DIV < 1)
#undef CTRL_DISTANCE_OUTER_DIV
#define CTRL_DISTANCE_OUTER_DIV 1
#endif

#ifndef CTRL_ANGLE_OUTER_DIV
#define CTRL_ANGLE_OUTER_DIV 1
#endif

#if (CTRL_ANGLE_OUTER_DIV < 1)
#undef CTRL_ANGLE_OUTER_DIV
#define CTRL_ANGLE_OUTER_DIV 1
#endif

void read_encoder(void);
void read_IMU(void);

void calculate_translation(void);
void calculate_rotation(void);

void velocity_PID(void);
void distance_PID(void);

void omega_PID(void);
void angle_PID(void);

void wall_PID(void);
void diagonal_CTRL(void);
void kushi_front_asym_CTRL(void);

#endif /* INC_CONTROL_H_ */
