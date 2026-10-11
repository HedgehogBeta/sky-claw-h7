/**
 * @file    DJmotor.h
 * @brief   DJI M2006/M3508 CAN motor driver, ported from R2 chassis.
 */
#ifndef DJMOTOR_H
#define DJMOTOR_H

#include <stdbool.h>
#include "main.h"
#include "pid.h"
#include "motor_cfg.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M3508_NUM MOTOR_M3508_COUNT
#define M2006_NUM MOTOR_M2006_COUNT
#define USE_DJNUM MOTOR_DJI_COUNT
#define M2006_RATIO MOTOR_M2006_REDUCTION_RATIO
#define M3508_RATIO MOTOR_M3508_REDUCTION_RATIO
#define Zero_Distance 15

    typedef enum
    {
        DJ_Disable = 0,  /* 关: transmit 0 current */
        DJ_RPM = 1,      /* 速度 mode          */
        DJ_Position = 2, /* 位置 mode       */
        DJ_Zero = 3,     /* 寻零 mode       */
        DJ_Current = 4,  /* 电流/扭矩        */

    } DJmotor_mode_t;

    typedef struct
    {
        volatile int16_t current_raw;  // 直接设置电流
        volatile float angle_deg;      // 输出角度, degree
        volatile float speed_rpm;      // 输出轴 rpm；位置模式下 valSet 用作速度前馈
        volatile float current_A;      // 反馈电流, A
        volatile int16_t PulseRead;    // raw encoder pulse
        volatile int16_t PulseGap;     // pulse delta
        volatile int32_t PulseTotal;   // accumulated pulse
        volatile int8_t temperature_C; // ℃
    } DJmotorVal;

    typedef struct
    {
        uint16_t PulsePerRound;   //           8191
        float Gear_ratio;         // mechanism ratio
        float Reduction_ratio;    // motor reducer ratio
        int16_t CurrentLimit_raw; // output current limit, raw
        float PositionSpeedFeedforward; // 位置模式速度前馈倍率，模板默认关闭
    } DJmotorParam;

    typedef struct
    {
        bool RPMLimitFlag;
        bool PosAngleLimitFlag;
        bool PosRPMFlag;
        bool PosDeadbandFlag;         // 允许位置死区，运动阶段由机构层关闭
        bool CurrentLimitFlag;
        float MaxAngle_deg;           // degree
        float MinAngle_deg;           // degree
        float PosDeadbandEnter_deg;   // 输出轴位置死区进入误差
        float PosDeadbandExit_deg;    // 输出轴位置死区退出误差，须大于进入值
        int16_t SpeedRPMLimit;        // rpm
        int32_t PosRPMLimit;          // rpm
        int16_t ZeroRPMLimit;         // rpm
        int16_t ZeroCurrentLimit_raw; // raw
        bool IsLooseStuck;
    } DJmotorLimit;

    typedef struct
    {
        volatile bool ZeroFlag;
        volatile bool Overtimeflag;
        volatile bool StuckFlag;
        volatile bool IsSetZero;
    } DJmotorStatus;

    typedef struct
    {
        volatile int32_t pulseLock; // 原先用于使能固定的字段,现在已无用
        float current_accum_raw;   // 闭环电流以浮点累加，下发 CAN 时才转换为整数
        bool pos_deadband_active;  // 位置死区滞回状态，不代表实际关节到位
        uint16_t zeroCnt;
        uint16_t GapCnt;
    } DJmotorArgum;

    typedef struct
    {
        volatile uint32_t lastRxTime;
        uint16_t stuckCount;
        uint16_t timeoutCount;
    } DJmotorError;

    typedef struct
    {
        PIDType posPID;
        PIDType velPID;
    } DJmotorPID;

    typedef struct
    {
        uint8_t ID;
        volatile bool Begin;              // true 运行 MODE;false 失能
        volatile DJmotor_mode_t MODE_Set; // DJ_Disable 即失能(发 0 电流)
        volatile DJmotor_mode_t MODE_Cur; // 实际运行模式,任务层可读

        DJmotorParam param;
        DJmotorVal valSet;
        DJmotorVal valNow;
        DJmotorVal valPre;
        DJmotorStatus statusFlag;
        DJmotorLimit limit;
        DJmotorArgum argum;
        DJmotorError error;
        PIDType posPID;
        PIDType velPID;
    } DJMotor, *DJMotorPointer;

#if USE_DJ
    extern DJMotor DJmotor[USE_DJNUM];

    void DJmotor_Init(void);
    void DJmotor_Func(void);
    void DJmotor_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);
    void DJmotor_PID_Reload(DJMotorPointer motor, DJmotorPID pid_reload);

#endif /* USE_DJ */

#ifdef __cplusplus
}
#endif

#endif /* DJMOTOR_H */
