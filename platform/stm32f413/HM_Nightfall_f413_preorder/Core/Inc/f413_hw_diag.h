#ifndef F413_HW_DIAG_H_
#define F413_HW_DIAG_H_

void f413_hw_diag_run_led_test_once(void);
void f413_hw_diag_run_switch_test_once(void);
void f413_hw_diag_run_buzzer_test_once(void);
void f413_hw_diag_run_fan_pwm_test_once(void);
void f413_hw_diag_run_motor_driver_test_once(void);
void f413_hw_diag_start_motor_break_in_continuous(void);
void f413_hw_diag_run_encoder_test_once(void);
/* RED: lifted/secured only, 7..9 V, bounded 6/12/18% motor sweep. */
void f413_hw_diag_run_lifted_sweep_once(void);

#endif
