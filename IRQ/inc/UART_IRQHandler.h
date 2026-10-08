/**
 * @file    UART_IRQHandler.h
 * @brief   UART HAL 中断回调(IRQ 层):按 huart 直接分发到各通信模块。
 *
 * 只放 UART 相关的 HAL 回调强定义(HAL 里是 __weak,这里强定义覆盖,
 * CubeMX 重新生成不冲突)。每个串口的接收缓冲 / DCache / DMA 重启
 * 由拥有它的模块自己负责,这里只做"判断是哪个串口 → 调对应模块函数"。
 *
 * 新增串口:在 UART_IRQHandler.c 对应的回调里加一个 else-if 分支即可,
 * 不需要注册表 / 设备表。
 */
#ifndef UART_IRQHANDLER_H
#define UART_IRQHANDLER_H

#include "main.h" /* UART_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/* 强定义覆盖 HAL 的 __weak 回调(原型与 stm32h7xx_hal_uart.h 一致) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* UART_IRQHANDLER_H */
