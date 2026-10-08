/**
 * @file    bluetooth.h
 * @brief   蓝牙串口通信模块(FreeRTOS 基础 Queue 版)。
 *
 * 职责分层:
 *   UART 中断   : HAL_UART_IRQHandler(CubeMX 生成)→ HAL 回调 → IRQ/UART_IRQHandler.c
 *                 按 huart 分发到本模块(本模块拥有 BT_UART 的接收缓冲,
 *                 自己处理 DCache 失效与 DMA 重启;串口映射见 FML/fm_uart_cfg.h)
 *   本模块      : 协议帧(包头/包尾/校验,CRC16 或单字节校验和可选)、
 *                 RX/TX Queue、收发任务、BT_UART 的 DMA+IDLE 接收
 *   业务层      : 只调用 Bluetooth_TaskCreate() / Bluetooth_Send() / Bluetooth_Receive()
 *
 * 业务层不需要知道 UART、DMA、IDLE、CRC、缓冲区的存在。
 * 蓝牙只传一种数据:业务层拿到 payload 后按自己的约定解析,模块不关心内容。
 *
 * 用法(在业务任务里):
 *   // 发送:组一个消息,丢给模块,剩下的事模块管
 *   uint8_t data[2] = {0x01, 0x02};
 *   Bluetooth_Send(data, 2);
 *
 *   // 接收:从 RX Queue 取一帧,超时返回 false
 *   BluetoothFrame_t frame;
 *   if (Bluetooth_Receive(&frame, 100)) {
 *       // frame.payload[0 .. frame.length-1] 就是本帧数据
 *   }
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <stdbool.h>

#include "bluetooth_serv.h" /* BT_Cmd_t / BT_Status_t(业务结构体 = 线上布局) */
#include "bluetooth_cfg.h"  /* 帧格式参数宏(包头/包尾/校验/包长字段) */
#include "usart.h"          /* UART_HandleTypeDef / huart6 声明 */

#include "app_cfg.h" /* APP_PRIO_* 任务优先级 */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 配置(按需修改;收发双方必须一致)                                        */
/* 帧格式参数宏(包头/包尾/校验/包长字段)在 bluetooth_cfg.h */
/* ------------------------------------------------------------------ */
#define BT_RX_QUEUE_SIZE        4U      /* 业务层接收队列深度(帧数) */
#define BT_TX_QUEUE_SIZE        4U      /* 发送队列深度(帧数) */
#define BT_SEND_TIMEOUT_MS      100U    /* Bluetooth_Send() 入队最长等待 */
#define BT_TASK_STACK_WORDS     200U    /* 收发任务栈大小(字) */
#define BT_RX_TASK_PRIORITY     APP_PRIO_ABOVE_NORMAL        /* 接收任务优先级*/
#define BT_TX_TASK_PRIORITY     APP_PRIO_NORMAL  /* 发送任务优先级 */

/* ------------------------------------------------------------------ */
/* 包体长度(取业务结构体 sizeof,不用手工维护)                            */
/* ------------------------------------------------------------------ */
#define BT_RX_PAYLOAD_SIZE ((uint16_t)sizeof(BT_Cmd_t))    /* 收(命令)包体字节数 */
#define BT_TX_PAYLOAD_SIZE ((uint16_t)sizeof(BT_Status_t)) /* 发(状态)包体字节数 */

/* 帧结构里包体缓冲的容量 = 两者较大 */
#define BT_PAYLOAD_MAX \
    ((BT_RX_PAYLOAD_SIZE > BT_TX_PAYLOAD_SIZE) ? BT_RX_PAYLOAD_SIZE : BT_TX_PAYLOAD_SIZE)

/* 启用包长字段时包体须 <= 255(包长字段只有 1 字节) */
#if BT_PAYLOAD_LENGTH > 0U
typedef char BT_STATIC_ASSERT_PAYLOAD_FITS_U8[
    ((BT_RX_PAYLOAD_SIZE <= 255U) && (BT_TX_PAYLOAD_SIZE <= 255U)) ? 1 : -1];
#endif

/* ------------------------------------------------------------------ */
/* 帧结构:Queue 里直接存整个结构体(按值拷贝),业务层不用管内存/生命周期     */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  payload[BT_PAYLOAD_MAX];         /* 包体缓冲:容量取 RX/TX 两者较大,实际长度见 length */
    uint16_t length;                          /* 包体有效字节数(1 .. 最大) */
} BluetoothFrame_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */
/* 建队列/建收发任务:启动调度器之前调用一次(main() 里,osKernelStart() 之前)。
   启动接收是模块内部的事(接收任务进循环前自己做),业务层不用管;
   为什么要分两步、以及中断里调 FreeRTOS API 的前提,见 DOCS/bluetooth.md。 */
void Bluetooth_TaskCreate(void);

bool Bluetooth_Send(const uint8_t *payload, uint16_t length);
bool Bluetooth_Receive(BluetoothFrame_t *frame, uint32_t timeout_ms);

/* 以下三个函数由 IRQ/UART_IRQHandler.c 的 UART HAL 回调按 huart 分发调用,业务层不要碰 */
void Bluetooth_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void Bluetooth_UartTxCplt(UART_HandleTypeDef *huart);
void Bluetooth_UartRxError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_H */