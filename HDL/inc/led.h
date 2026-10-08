/**
 * @file    led.h
 * @brief   LED 设备接口(HDL 层,引脚映射见 HDL/inc/bsp_board_config.h)。
 */
#ifndef LED_H
#define LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Led_Init(void);
void Led_On(uint8_t index);
void Led_Off(uint8_t index);
void Led_Toggle(uint8_t index);
void Led_Set(uint8_t index, uint8_t on);
void Led_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
