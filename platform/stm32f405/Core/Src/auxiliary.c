/*
 * auxiliary.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "drive.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
// led_write
// LEDを点灯させる
// 引数1：led1 …… led1のON/OFF
// 引数2：led2 …… led2のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void led_write(bool led_l, bool led_r, bool led_f) {
    if (led_l) {
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    }

    if (led_r) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    }

    if (led_f) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// led_flash
// LEDを点滅させる
// 引数1：times …… 点滅回数
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++

void led_flash(uint8_t times) {
    for (int i = 0; i < times; i++) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

        HAL_Delay(70);

        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

        HAL_Delay(70);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// led_wait
// LEDを点滅させながら無限ループで待機
// 引数1：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void led_wait(void) {
    while (1) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

        HAL_Delay(400);

        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);

        HAL_Delay(400);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// buzzer_beep
// ブザーを鳴らす
// 引数1：tone
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++

void buzzer_beep(uint16_t tone) {
    // ファン停止直後はクールダウン中はブザーを抑止（TIM3共有による干渉回避）
    const uint32_t now_ms = HAL_GetTick();
    if (!drive_should_suppress_buzzer() &&
        (fan_last_off_ms == 0u || (now_ms - fan_last_off_ms) > 200u)) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        __HAL_TIM_SET_AUTORELOAD(&htim3, tone);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, tone * 0.6);
        HAL_Delay(200);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// buzzer_interrupt
// ブザーを割込中で鳴らす
// 引数1：tone
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++

void buzzer_interrupt(uint16_t tone) {
    // ファン停止直後はクールダウン中はブザーを抑止（TIM3共有による干渉回避）
    if (!drive_should_suppress_buzzer()) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        __HAL_TIM_SET_AUTORELOAD(&htim3, tone);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, tone * 0.5);
        buzzer_count = 150;  // 発音時間を延長（150ms）
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// buzzer_enter
// ブザーを短く2回鳴らす
// 引数1：tone
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++

void buzzer_enter(uint16_t tone) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_AUTORELOAD(&htim3, tone);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, tone * 0.7);
    HAL_Delay(100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_Delay(50);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, tone * 0.7);
    HAL_Delay(100);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

/*------------------------------------------------------------
    モード選択
------------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
// select_mode
// モード選択を行う
// 引数1：モード番号の初期値
// 戻り値：選択されたモード番号
//+++++++++++++++++++++++++++++++++++++++++++++++
int select_mode(int mode) {
    while (1) {
        if (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == 0) {
            mode++;

            if (mode > 9) {
                mode = 0;
            }

            printf("Mode : %d\n", mode);

            buzzer_beep((11 - mode) * 0.5 * 800);
        }

        if (ad_fr > WALL_BASE_FR && ad_fl < WALL_BASE_FL) {
            buzzer_enter(900);
            return mode;
        }
    }

    printf("Mode : %d\n", mode);
}

/*------------------------------------------------------------
    printf 用
------------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//__io_putchar
// printf を使うために必要
//+++++++++++++++++++++++++++++++++++++++++++++++
/*
int __io_putchar(int c) {
  if (c == '\n') {
    int _c = '\r';
    HAL_UART_Transmit(&huart1, &_c, 1, 1);
  }
  HAL_UART_Transmit(&huart1, &c, 1, 1);
  return 0;
}
*/

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void __io_putchar(int ch) { 
    uint8_t temp = ch;
    HAL_UART_Transmit(&huart1, &temp, 1, 1); 
}
