/**
 * @file    buzzer.c
 * @brief   蜂鸣器设备实现(PB0,引脚见 bsp_board_config.h,归 HDL 层)。
 */
#include "buzzer.h"
#include "bsp_board_config.h"

void Buzzer_Init(void)
{
    Buzzer_Off();
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN, BOARD_BUZZER_ACTIVE_LEVEL);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN,
                      (BOARD_BUZZER_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Buzzer_Toggle(void)
{
    HAL_GPIO_TogglePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
}

void Buzzer_StartupBeep(void)
{
    Buzzer_On();
    HAL_Delay(50);
    Buzzer_Off();
    HAL_Delay(50);
    Buzzer_On();
    HAL_Delay(50);
    Buzzer_Off();
    HAL_Delay(50);
}

void Buzzer_Alarm(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        Buzzer_On();
        HAL_Delay(on_ms);
        Buzzer_Off();
        HAL_Delay(off_ms);
    }
}
