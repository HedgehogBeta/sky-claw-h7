#include "main.h" /* FDCAN_HandleTypeDef */

#ifndef EXTI_IRQHandler_H
#define EXTI_IRQHandler_H

#ifdef __cplusplus
extern "C"
{
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#ifdef __cplusplus
}
#endif

#endif /* EXTI_IRQHandler_H */