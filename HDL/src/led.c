/**
 * @file    led.c
 * @brief   LED 设备实现(PB3..PB6 流水灯,引脚见 bsp_board_config.h,归 HDL 层)。
 */
#include "led.h"
#include "bsp_board_config.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} Led_Device_t;

static Led_Device_t s_led_device[BOARD_LED_NUM] = {
    { BOARD_LED1_PORT, BOARD_LED1_PIN },
    { BOARD_LED2_PORT, BOARD_LED2_PIN },
    { BOARD_LED3_PORT, BOARD_LED3_PIN },
    { BOARD_LED4_PORT, BOARD_LED4_PIN },
};

void Led_Init(void)
{
    Led_AllOff();
}

void Led_On(uint8_t index)
{
    if (index < BOARD_LED_NUM) {
        HAL_GPIO_WritePin(s_led_device[index].port, s_led_device[index].pin, GPIO_PIN_SET);
    }
}

void Led_Off(uint8_t index)
{
    if (index < BOARD_LED_NUM) {
        HAL_GPIO_WritePin(s_led_device[index].port, s_led_device[index].pin, GPIO_PIN_RESET);
    }
}

void Led_Toggle(uint8_t index)
{
    if (index < BOARD_LED_NUM) {
        HAL_GPIO_TogglePin(s_led_device[index].port, s_led_device[index].pin);
    }
}

void Led_Set(uint8_t index, uint8_t on)
{
    if (on != 0U) {
        Led_On(index);
    } else {
        Led_Off(index);
    }
}

void Led_AllOff(void)
{
    for (uint8_t i = 0; i < BOARD_LED_NUM; i++) {
        Led_Off(i);
    }
}

void Led_Flow(uint32_t period_ms)
{
    for (uint8_t i = 0; i < BOARD_LED_NUM; i++) {
        Led_On(i);
        HAL_Delay(period_ms);
        Led_Off(i);
        HAL_Delay(period_ms);
    }
}
