/**
 * @file    UnitreeMotor.h
 * @brief   Unitree GO-M8010-6 RS485 motor driver.
 *
 * The GO motor protocol layer (wire frames and runtime command/feedback
 * structures) is kept in this header so the driver does not depend on a
 * separate protocol header.
 *
 * NOTE: GO-M8010-6 has an absolute encoder. This driver does NOT perform
 * auto zero on power-up. At the mechanical zero, keep enable=false and set
 * set_zero=true; a fresh feedback frame is required before zero is captured.
 */
#ifndef UNITREEMOTOR_H
#define UNITREEMOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "motor_cfg.h"
#include "crc_ccitt.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define UNITREE_MOTOR_NUM MOTOR_UNITREE_COUNT
#define UNITREE_GEAR_RATIO MOTOR_UNITREE_REDUCTION_RATIO
#define UNITREE_FRAME_LENGTH 16U

    typedef enum
    {
        UNITREE_MOTOR_MODE_IDLE = 0, /* 0: 锁定/空闲 */
        UNITREE_MOTOR_MODE_FOC = 1,  /* 1: FOC 闭环    */
        UNITREE_MOTOR_MODE_CALIB = 2 /* 2: 编码器校准  */
    } UnitreeMotorMode_t;

/* ------------------------------------------------------------------ */
/* 协议层:线缆帧(必须 1 字节对齐)                                       */
/* ------------------------------------------------------------------ */
#pragma pack(1)

    typedef struct
    {
        uint8_t id : 4;      /* 电机 ID: 0..14, 15 为广播(无返回) */
        uint8_t status : 3;  /* 工作模式: 0.锁定 1.FOC 2.校准 */
        uint8_t reserve : 1; /* 保留 */
    } UnitreeMotorWireMode_t;

    typedef struct
    {
        int16_t torque_des; /* 转子力矩, N.m (q8) */
        int16_t speed_des;  /* 转子转速, rev/s (q8) */
        int32_t pos_des;    /* 转子转数, rev (q15) */
        int16_t kp;         /* 转子侧协议刚度增益 */
        int16_t kd;         /* 转子侧协议阻尼增益 */
    } UnitreeMotorWireCmd_t;

    typedef struct
    {
        int16_t torque;           /* 转子力矩, N.m (q8) */
        int16_t speed;            /* 转子转速, rev/s (q8) */
        int32_t pos;              /* 转子转数, rev (q15) */
        int8_t temp;              /* 电机温度, -128~127 ℃ */
        uint8_t error : 3;        /* 错误标识 */
        uint16_t foot_force : 12; /* 足端力/气压传感器 12bit */
        uint8_t none : 1;         /* 保留 */
    } UnitreeMotorWireFbk_t;

    typedef struct
    {
        uint8_t head[2]; /* 包头 0xFE 0xEE */
        UnitreeMotorWireMode_t mode;
        UnitreeMotorWireCmd_t comd;
        uint16_t crc16;
    } UnitreeMotorControlFrame_t;

    typedef struct
    {
        uint8_t head[2]; /* 包头 0xFD 0xEE */
        UnitreeMotorWireMode_t mode;
        UnitreeMotorWireFbk_t fbk;
        uint16_t crc16;
    } UnitreeMotorFeedbackFrame_t;

#pragma pack()

/* ------------------------------------------------------------------ */
/* 运行时结构体(含 float,保持 4 字节对齐)                                */
/* ------------------------------------------------------------------ */
#pragma pack(4)

    typedef enum{
        UNITREE_NO_ERR = 0,
        UNITREE_OVER_HEAT = 1,
        UNITREE_OVER_CURRENT = 2,
        UNITREE_OVER_VOLTAGE = 3,
        UNITREE_ENC_ERR = 4,
    }UnitreeMotorError_t;

    typedef struct
    {
        uint16_t id;    /* 电机 ID, 15 为广播 */
        uint16_t mode;  /* UnitreeMotorMode_t */
        float torque;   /* 期望减速器输出轴力矩, N.m */
        float speed;    /* 期望减速器输出轴速度, rad/s */
        float position; /* 期望减速器输出轴位置, rad */
        float kp;       /* 转子侧协议刚度增益 0~25.599 */
        float kd;       /* 转子侧协议阻尼增益 0~25.599 */
        UnitreeMotorControlFrame_t frame;
    } UnitreeMotorCmd_t;

    typedef struct
    {
        uint8_t id;        /* 电机 ID */
        uint8_t mode;      /* UnitreeMotorMode_t */
        int temperature;   /* 温度 ℃ */
        UnitreeMotorError_t error;         /* 错误码 */
        float torque;      /* 实际关节输出力矩, N.m */
        float speed;       /* 实际关节输出速度, rad/s */
        float position;    /* 实际关节输出位置, rad(已减去零位偏移) */
        int correct;       /* 1=有效反馈；超过100ms无新帧时置0 */
        int foot_force;    /* 足端力原始值 */
        uint16_t calc_crc; /* 本机计算的 CRC */
        uint32_t timeout;  /* 反馈超时或接收错误恢复次数 */
        uint32_t bad_msg;  /* CRC/包头错误计数 */
        UnitreeMotorFeedbackFrame_t frame;
    } UnitreeMotorData_t;

    typedef struct
    {
        volatile bool enable;   /* true=发送 FOC 模式, false=发送空闲模式 */
        volatile bool set_zero; /* 失能且反馈新鲜时，将当前位置设为零点 */
        bool begin;             /* true=参与周期发送 */

        UnitreeMotorCmd_t cmd;   /* 用户写目标值 */
        UnitreeMotorData_t data; /* 最新反馈(已修正零位) */
        float zero_offset;       /* 控制坐标系下的零位偏移 */
    } UnitreeMotor;

#pragma pack()

#if USE_UNITREE
    extern UnitreeMotor Unitree_motors[UNITREE_MOTOR_NUM];

    void UnitreeMotor_Init(void);
    void UnitreeMotor_Func(void);

    /* 以下三个函数由 IRQ/UART_IRQHandler.c 的 UART HAL 回调按 huart 分发调用,业务层不要碰。
       UART7 是 RS485 半双工:发完一帧命令 → 方向切回接收,应答由环形 DMA 双缓冲 +
       IDLE 空闲中断自动接收(UnitreeMotor_UART_RxEvent 里取最近一帧解析)。
       RS485 方向控制(DE 引脚)归本驱动所有(引脚定义见 bsp_board_config.h)。 */
    void UnitreeMotor_UART_TxCpltCallback(UART_HandleTypeDef *huart);
    void UnitreeMotor_UART_RxEvent(UART_HandleTypeDef *huart);
    void UnitreeMotor_UART_ErrorCallback(UART_HandleTypeDef *huart);
#endif /* USE_UNITREE */

#ifdef __cplusplus
}
#endif

#endif /* UNITREEMOTOR_H */
