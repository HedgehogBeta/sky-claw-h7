/**
 * @file    bluetooth_cfg.h
 * @brief   蓝牙通信层配置宏(归 FML/bluetooth)。
 *
 * 这里集中放蓝牙模块的可调宏(帧格式参数),改参数不用去翻协议实现文件。
 *
 * 注意:Payload 字段与包体长度不在此手工填写,而是由 bluetooth_serv.h 的
 * pack(1) 业务结构体(BT_Cmd_t / BT_Status_t)的 sizeof 决定,长度宏在 bluetooth.h。
 */
#ifndef BLUETOOTH_CFG_H
#define BLUETOOTH_CFG_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* 蓝牙帧格式参数宏(bluetooth.c 计算宏据此推导)                           */
/* 与上位机约定,收发双方一致,编译期定死。                                 */
/* ------------------------------------------------------------------ */
#define BT_FRAME_HEADER1    0xA5            /* 包头第 1 字节 */
#define BT_FRAME_HEADER2    0x00            /* 包头第 2 字节(设为 0 = 只用包头1,单包头) */
#define BT_FRAME_TAIL1      0x5A            /* 包尾第 1 字节 */
#define BT_FRAME_TAIL2      0x00            /* 包尾第 2 字节(设为 0 = 只用包尾1,单包尾) */
#define BT_CHECKSUM_MODE    1U              /* 校验方式:0 = CRC16, 1 = 单字节校验和 */
#define BT_PAYLOAD_LENGTH   0U              /* 是否带 1 字节包长字段:0 = 无包长字段,包头后直接跟包体;
                                               1..N = 带 1 字节包长字段(写的是该方向的包体长度)。
                                               包体长度本身按方向区分,由 bluetooth_serv.h 的
                                               pack(1) 结构体 sizeof 决定,不在此配置 */

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_CFG_H */
