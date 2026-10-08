/**
 * @file    TIM_IRQHandler.c
 * @brief   定时器周期中断分发 + 电机调度(原 Application/src/IRQHandler.c 内容并入)。
 *
 * 替代已删除的 bsp_tim:原来由 BspTim_RegisterCallback/BspTim_Dispatch 分发的
 * 逻辑,现在直接在这里按定时器分发(电机调度内联在 TIM2 分支):
 *   TIM2 : 1kHz 控制定时器(CAN 出队 + 电机 Func);
 *   TIM4 : 200Hz 控制定时器(ZdriveFunc)。
 *   TIM3/5 备用,需要时在这里加分支。
 *
 * 调用链(自动,无需手动注册):
 *   TIMx 中断 → HAL_TIM_IRQHandler → main.c 的 HAL_TIM_PeriodElapsedCallback
 *   (CubeMX 生成:TIM1 时基 HAL_IncTick)→ USER CODE Callback 1 区
 *   → 本文件的 USER_TIM_PeriodElapsedCallback()(TIM2/TIM4 分支直接调度)。
 *
 * 注意:TIM1(HAL 时基)由 main.c 的 CubeMX 代码处理,这里不要再碰,
 * 否则 uwTick 会重复递增。
 */
#include "TIM_IRQHandler.h"

#include "includes.h"
#include "motor_cfg.h"
#include "can_queue.h"
#include "ZDrive.h"
#include "DJmotor.h"
#include "VescMotor.h"
#include "UnitreeMotor.h"
#include "mechanism_app.h"

void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
        /* 1kHz 控制定时器:电机调度*/
        if (htim->Instance == TIM2)
        {
                Mechanism_TimerCallback();
                CAN_DequeueTx(&CAN1_Txqueue);
#if USE_ZMDR
                ZdriveDequeue((uint8_t)MOTOR_ZDRIVE_CAN_BUS_1);
                ZdriveDequeue((uint8_t)MOTOR_ZDRIVE_CAN_BUS_2);
#endif
#if USE_DJ
                DJmotor_Func();
#endif
#if USE_UNITREE
                UnitreeMotor_Func();
#endif
        }
        // 200Hz控制定时器
        else if (htim->Instance == TIM4)
        {
#if USE_ZMDR
                ZdriveFunc();
#endif
        }
}
