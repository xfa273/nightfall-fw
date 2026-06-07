#ifndef F413_IMU_DIAG_H_
#define F413_IMU_DIAG_H_

#include <stdbool.h>
#include <stdint.h>

bool f413_imu_diag_read_reg(uint8_t reg, uint8_t* out);
bool f413_imu_diag_whoami_ok(void);
void f413_imu_diag_run_accel_test_once(void);
void f413_imu_diag_run_manual_turn_test_once(void);
void f413_imu_diag_run_whoami_test_once(void);

#endif
