/**
 * @file    CAN_IRQHandler.h
 * @brief   FDCAN HAL 接收中断回调(IRQ 层):按 hfdcan 分发到各电机驱动。
 *
 * 接收不走软件队列 —— 反馈帧在 FDCAN 接收中断里直接解析
 * (FDCAN 与 TIM2 同为优先级 5,同优先级互不抢占,与 TIM2 里 Func 读电机
 * 状态天然互斥)。每个驱动的 Receive 内部都按帧 ID 过滤。
 */
#ifndef CAN_IRQHANDLER_H
#define CAN_IRQHANDLER_H

#include "main.h" /* FDCAN_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/* 强定义覆盖 HAL 的 __weak 回调(原型与 stm32h7xx_hal_fdcan.h 一致) */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

#ifdef __cplusplus
}
#endif

#endif /* CAN_IRQHANDLER_H */
