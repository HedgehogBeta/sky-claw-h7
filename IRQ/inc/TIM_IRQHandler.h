/**
 * @file    TIM_IRQHandler.h
 * @brief   用户定时器周期中断分发(IRQ 层)。
 *
 * 分工:
 *   HAL_TIM_PeriodElapsedCallback(CubeMX 生成,在 main.c):
 *     TIM1 时基 → HAL_IncTick();USER CODE Callback 1 区自动调用本文件函数。
 *   USER_TIM_PeriodElapsedCallback(本文件):
 *
 * 这样 CubeMX 重新生成 main.c 不会丢代码 —— HAL 回调留在 main.c,
 * 用户分发在 USER CODE 区调用,IRQ 层只提供被调用的分发函数。
 */
#ifndef TIM_IRQHANDLER_H
#define TIM_IRQHANDLER_H

#include "main.h" /* TIM_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/* 由 main.c 的 HAL_TIM_PeriodElapsedCallback自动调用 */
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* TIM_IRQHANDLER_H */
