/**
 * @file    bsp_board_config.h
 * @brief   板级硬件映射(固定板级资源:引脚)。
 *
 * 只放"永远存在"的板级资源:LED/蜂鸣器/RS485 引脚。
 * 串口接收缓冲、串口句柄归属各通信模块(谁用串口谁定义,直接引用 usart.h)。
 */
#ifndef BSP_BOARD_CONFIG_H
#define BSP_BOARD_CONFIG_H

#include "main.h" /* GPIO 类型 / GPIOB / GPIO_PIN_x */

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* LED                                                                  */
/* ------------------------------------------------------------------ */
#define BOARD_LED_NUM 4U

#define BOARD_LED1_PORT GPIOB
#define BOARD_LED1_PIN GPIO_PIN_3
#define BOARD_LED2_PORT GPIOB
#define BOARD_LED2_PIN GPIO_PIN_4
#define BOARD_LED3_PORT GPIOB
#define BOARD_LED3_PIN GPIO_PIN_5
#define BOARD_LED4_PORT GPIOB
#define BOARD_LED4_PIN GPIO_PIN_6

/* ------------------------------------------------------------------ */
/* Buzzer                                                               */
/* ------------------------------------------------------------------ */
#define BOARD_BUZZER_PORT GPIOB
#define BOARD_BUZZER_PIN GPIO_PIN_0
#define BOARD_BUZZER_ACTIVE_LEVEL GPIO_PIN_SET

/* ------------------------------------------------------------------ */
/* RS485 收发方向控制(DE 引脚)                                          */
/* ------------------------------------------------------------------ */
#define BOARD_RS485_U7_PORT GPIOG
#define BOARD_RS485_U7_PIN  GPIO_PIN_1

#ifdef __cplusplus
}
#endif

#endif /* BSP_BOARD_CONFIG_H */
