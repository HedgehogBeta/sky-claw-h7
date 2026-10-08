/**
 * @file    can_queue.h
 * @brief   FDCAN TX 发送队列(硬件驱动层基础设施,原 bsp_can.h)。
 *
 * 从 Motor/FD_Canqueue.h 搬迁而来,职责不变:只负责发送队列。
 * 驱动入队(ZdriveEnqueue / VESC 直接写队列),TIM2 中断按 1kHz 出队。
 * 接收不走软件队列 —— 各电机反馈帧在 FDCAN 接收中断里直接解析。
 *
 */
#ifndef CAN_QUEUE_H
#define CAN_QUEUE_H

#include <stdbool.h>
#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define FDCAN_QUEUESIZE 64U

    typedef struct
    {
        uint32_t ID;
        uint8_t DLC;
        uint32_t IDE;
        uint8_t Data[8];
        bool InConGrpFlag;
    } FDCAN_DataStruct;

    typedef struct
    {
        uint8_t Front;
        uint8_t Rear;
        FDCAN_GlobalTypeDef *Canx;
        FDCAN_DataStruct FDCAN_DataSend[FDCAN_QUEUESIZE];
    } FDCAN_SendQueueType;

    extern FDCAN_SendQueueType CAN1_Txqueue;
    extern FDCAN_SendQueueType CAN2_Txqueue;
    extern FDCAN_SendQueueType CAN3_Txqueue;

    void CAN_InitSendQueue(void);
    /** FDCAN 启动:全局滤波 + FIFO 覆盖 + Start + RX 通知(.ioc 未开 Start,统一在这里做) */
    void CAN_Start(void);
    bool CAN_Queue_IfEmpty(FDCAN_SendQueueType *queue);
    bool CAN_Queue_IfFull(FDCAN_SendQueueType *queue);
    bool CAN_DequeueTx(FDCAN_SendQueueType *queue);
    void CAN_Enqueue(FDCAN_SendQueueType *queue, FDCAN_RxHeaderTypeDef Rxheader, uint8_t Rxdata[]);
    void HeaderPrepare(uint32_t sendCode, uint32_t datalen, FDCAN_RxHeaderTypeDef *rxheader);

#ifdef __cplusplus
}
#endif

#endif /* CAN_QUEUE_H */
