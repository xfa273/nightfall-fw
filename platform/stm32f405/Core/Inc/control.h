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

#endif /* INC_CONTROL_H_ */
