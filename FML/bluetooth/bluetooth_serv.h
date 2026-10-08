/**
 * @file    bluetooth_serv.h
 * @brief   蓝牙业务消息层:命令/状态结构体 + 收发接口。
 *
 * 结构体即线上包体布局:两个结构体都用 #pragma pack(1) 打包(无填充字节),
 * 收发各一次 memcpy 完成封装/解析。
 *
 * 字段类型:bool(flag 位域)/ int8_t / int16_t / int32_t / float,小端。
 * flag 用 bool : 1 位域写在结构体开头(8 个占 1 字节,不足 8 个也占满整字节)。
 *
 * 改协议只改下面的结构体字段(增删 / 改名 / 调顺序);包体长度/帧长这类宏在帧层
 * bluetooth.h(BT_*_PAYLOAD_SIZE 由结构体 sizeof 推出)。
 */
#ifndef BLUETOOTH_SERV_H
#define BLUETOOTH_SERV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* 业务结构体 = 线上包体布局                                            */
/* ------------------------------------------------------------------ */
#pragma pack(1)

    /* 接收(上位机 → 板子) */
    typedef struct
    {
        bool enable : 1;
        bool auto_mode : 1;
        bool calib : 1;
        bool stop : 1;
        bool slow : 1;
        bool fast : 1;
        bool reset : 1;
        int16_t speed;
        int16_t angle;
        float vx;
    } BT_Cmd_t;

    /* 发送(板子 → 上位机) */
    typedef struct
    {
        bool online : 1;
        bool ready : 1;
        int16_t rpm;
        float current;
    } BT_Status_t;

#pragma pack()

/* 布局按小端定义,不支持大端目标 */
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "蓝牙包体布局按小端定义,不支持大端目标。"
#endif

    /* ------------------------------------------------------------------ */
    /* 接口                                                                */
    /* ------------------------------------------------------------------ */
    /* 发状态:BT_Status_t → 包体 → Bluetooth_Send(),返回入队结果 */
    bool BT_Serv_Status_Send(const BT_Status_t *status);

    /* 收命令:Bluetooth_Receive() → 包体 → BT_Cmd_t,没收到帧返回 false */
    bool BT_Serv_Cmd_Receive(BT_Cmd_t *cmd, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_SERV_H */
