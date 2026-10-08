/**
 * @file    CAN_IRQHandler.c
 * @brief   FDCAN 接收中断回调:各电机驱动的反馈帧全部在这里直接解析。
 *
 * 不再经过软件 RX 队列。FDCAN 与 TIM2 同为优先级 5,Cortex-M 同优先级互不
 * 抢占,Receive 写电机状态与 TIM2 里 Func 读电机状态天然互斥,无需加锁。
 * 每个驱动的 Receive 内部都按帧 ID 过滤,不是自己的帧自然丢弃。
 * 总线下标 = FDCAN 编号:传给驱动的 bus 分别是 FDCAN1→1、FDCAN2→2、FDCAN3→3。
 * 新增总线/驱动:在这里按 hfdcan 加分支即可。
 */
#include "CAN_IRQHandler.h"

#include "includes.h"
#include "motor_cfg.h"
#include "ZDrive.h"
#include "DJmotor.h"
#include "VescMotor.h"
#include "fdcan.h" /* hfdcan1/2/3 声明 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static uint8_t s_full_conter = 0U;
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef Rxheader;
        uint8_t Rx_data[8] = {0};
        /* 读取 FIFO0 的新消息,并清除中断标志 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rxheader, Rx_data);

        if (hfdcan->Instance == FDCAN1)
        {
#if USE_ZMDR
            ZdriveReceive(Rxheader, Rx_data, 1U);
#endif
#if USE_VESC && (MOTOR_VESC_CAN_BUS == 1U)
            VescReceiveData_CAN2(Rxheader, Rx_data);
            return;
#endif
#if USE_DJ && (MOTOR_DJI_CAN_BUS == 1U)
            DJmotor_Receive(Rxheader, Rx_data);
            return;
#endif
            /* CAN1 预留给用户通信,其余帧在这里直接解析 */
        }
        else if (hfdcan->Instance == FDCAN2)
        {
#if USE_ZMDR
            ZdriveReceive(Rxheader, Rx_data, 2U);
#endif
#if USE_VESC && (MOTOR_VESC_CAN_BUS == 2U)
            VescReceiveData_CAN2(Rxheader, Rx_data);
            return;
#endif
#if USE_DJ && (MOTOR_DJI_CAN_BUS == 2U)
            DJmotor_Receive(Rxheader, Rx_data);
            return;
#endif
        }
        else if (hfdcan->Instance == FDCAN3)
        {
#if USE_ZMDR
            ZdriveReceive(Rxheader, Rx_data, 3U);
#endif
#if USE_VESC && (MOTOR_VESC_CAN_BUS == 3U)
            VescReceiveData_CAN2(Rxheader, Rx_data);
            return;
#endif
#if USE_DJ && (MOTOR_DJI_CAN_BUS == 3U)
            DJmotor_Receive(Rxheader, Rx_data);
            return;
#endif
        }
    }
    else if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)
    {
        /* FIFO0 满了,清除中断标志,丢弃最旧的帧 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, NULL, NULL);
        s_full_conter++;
    }
}
// FIFO1 暂时不启用,可用于区分同一总线的不同电机
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    static uint8_t s_full_conter = 0U;
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef Rxheader;
        uint8_t Rx_data[8] = {0};
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &Rxheader, Rx_data);
        if (hfdcan->Instance == FDCAN1)
        {
        }
        else if (hfdcan->Instance == FDCAN2)
        {
        }
        else if (hfdcan->Instance == FDCAN3)
        {
        }
    }
    else if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)
    {
        s_full_conter++;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, NULL, NULL);
    }
}
