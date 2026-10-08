/**
 * @file    fm_uart_cfg.h
 * @brief   串口角色 → HAL 句柄的语义映射(FML 层统一配置)。
 *
 * 蓝牙 / IMU / VOFA 三个串口模块在代码里**只用语义宏**(BT_UART / IMU_UART /
 * VOFA_UART),不直接写 &huartX。换串口只需改本文件一行 + CubeMX 里改引脚。
 *
 *   - 接收缓冲 / DCache / DMA 重启仍在各模块内部管理(见各模块 .c);
 *     接收缓冲**大小**也在这里配置(见下方 *_UART_RX_SIZE);
 *   - 中断分发(IRQ/UART_IRQHandler.c)也用这里的宏比较,改一处全链路生效;
 *   - Unitree 电机走 RS485,其串口由 FML/motor/motor_cfg.h 的
 *     MOTOR_UNITREE_UART 决定(当前 UART7),不在这里配置。
 *
 * 用法:宏是外设对象(如 BT_UART == huart6),调用处写 &BT_UART。
 * 新增串口模块:在本文件加语义宏与接收缓冲大小,模块代码与 IRQ 分发里引用它即可。
 */
#ifndef FM_UART_CFG_H
#define FM_UART_CFG_H

#include "usart.h" 

#ifdef __cplusplus
extern "C" {
#endif

#define BT_UART   (huart6)

#define IMU_UART  (huart1)

#define VOFA_UART (huart9)

/* 新模块在这里加,如:#define XXX_UART (huartX) */

/* ------------------------------------------------------------------ */
/* 接收缓冲大小(字节):各模块的 DMA 接收缓冲数组按此分配,改这里即可。      */
/* ------------------------------------------------------------------ */
#define BT_UART_RX_SIZE   256U  /* 蓝牙:一段空闲最多收这么多 */
#define IMU_UART_RX_SIZE  272U  /* IMU:>= Yesense 最大帧长 262(7 + 255) */

#ifdef __cplusplus
}
#endif

#endif /* FM_UART_CFG_H */
