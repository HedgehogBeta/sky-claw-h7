/**
 * @file    UART_IRQHandler.c
 * @brief   UART HAL 中断回调:按 huart 直接分发到对应通信模块。
 *
 * CubeMX 生成的这些回调是 __weak,这里强定义覆盖,重新生成不冲突。
 * 原则:这里只做"判断是哪个串口 → 调对应模块自己的处理函数"。
 * 接收缓冲、DCache 失效、DMA 重启都由模块自己负责 —— 每个串口允许
 * 不同的接收方式:
 *   IMU(IMU_UART)    : DMA + IDLE + 单缓冲(收一段解析一段,模块内重启)
 *   BT(BT_UART)      : 同上
 *   UART7(Unitree)   : RS485 半双工,环形 DMA 双缓冲 + IDLE 收 16 字节应答
 *   VOFA(VOFA_UART)  : 只发不收,不接任何回调(发送任务自己看 gState 判空闲)
 * 串口→句柄的语义映射统一在 FML/fm_uart_cfg.h,换串口改那里一行即可,
 * 本文件的分支用宏比较会自动跟随;新增串口在下面加一个 else-if 分支即可,
 * 不需要任何注册表/设备表。
 */
#include "UART_IRQHandler.h"

#include "fm_uart_cfg.h" /* BT_UART / IMU_UART / VOFA_UART(内含 usart.h) */
#include "bluetooth.h"  /* Bluetooth_UartRxEvent / TxCplt / RxError */
#include "imu.h"        /* Imu_UartRxEvent / RxError */
#include "UnitreeMotor.h" /* UnitreeMotor_UART_*Callback */
#include "motor_cfg.h" /* USE_UNITREE */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == IMU_UART.Instance)
    {
        Imu_UartRxEvent(huart, Size);
    }
    else if (huart->Instance == BT_UART.Instance)
    {
        Bluetooth_UartRxEvent(huart, Size);
    }
    else if (huart->Instance == huart7.Instance) /* Unitree:环形 DMA 双缓冲 + IDLE 收应答 */
    {
#if USE_UNITREE
        UnitreeMotor_UART_RxEvent(huart);
#endif
    }
    /* 新串口(用 DMA+IDLE 接收)在这里加分支 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 目前没有模块用定长接收(Receive_DMA/Receive_IT);需要时在这里加分支 */
    (void)huart;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BT_UART.Instance)
    {
        Bluetooth_UartTxCplt(huart);
    }
    else if (huart->Instance == huart7.Instance) /* Unitree:见 motor_cfg.h 的 MOTOR_UNITREE_UART */
    {
#if USE_UNITREE
        UnitreeMotor_UART_TxCpltCallback(huart);
#endif
    }
    /* 新串口(要发数据)在这里加分支;VOFA 不需要:它只看 huart->gState 判空闲 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 机械部分:清 ORE/FE 错误标志(所有串口通用,先清掉再交给模块) */
    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
    if (huart->ErrorCode & HAL_UART_ERROR_FE)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
    }

    /* 策略部分:各模块决定怎么恢复自己的接收 */
    if (huart->Instance == IMU_UART.Instance)
    {
        Imu_UartRxError(huart);
    }
    else if (huart->Instance == BT_UART.Instance)
    {
        Bluetooth_UartRxError(huart);
    }
    else if (huart->Instance == huart7.Instance) /* Unitree:见 motor_cfg.h 的 MOTOR_UNITREE_UART */
    {
#if USE_UNITREE
        UnitreeMotor_UART_ErrorCallback(huart);
#endif
    }
    /* 未分发的串口(如 USART2/3/4)出错:忽略,由使用者自行决定 */
}
