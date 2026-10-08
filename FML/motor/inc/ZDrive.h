/**
 * @file    ZDrive.h
 * @brief   ZDrive J60/Z-Smart motor CAN driver, ported from R2 chassis.
 */
#ifndef ZDRIVE_H
#define ZDRIVE_H

#include <stdbool.h>
#include <stdlib.h>
#include "main.h"
#include "can_queue.h"
#include "motor_cfg.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define USE_ZDRIVE_NUM MOTOR_ZDRIVE_COUNT

#define POU 10000.f
#define POD -10000.f
#define Velocity_Limit 150.f
#define Current_Limit 40.f

    typedef enum
    {
        Zdrive_Disable = 0,              // 0失能
        Zdrive_Current,                  // 1扭矩模式
        Zdrive_Speed,                    // 2速度模式
        Zdrive_Postion,                  // 3位置模式
        Zdrive_Test,                     // 4测试模式
        Zdrive_RVCalibration,            // 5电阻电感校准
        Zdrive_EncoderLineCalibration,   // 6编码器线性补偿
        Zdrive_EncoudeOffsetCalibration, // 7编码器偏移校准
        Zdrive_VKCalibration,            // 8VK校准
        Zdrive_SaveSetting,              // 9保存配置
        Zdrive_EraseSetting,             // 10擦除配置
        Zdrive_ClearErr,                 // 11擦除错误
        Zdrive_Brake                     // 12刹车
    } ZdriveMode;

    typedef enum
    {
        Zdrive_Well = 0,            // 0无
        Zdrive_InsufficientVoltage, // 1低电压
        Zdrive_OverVoltage,         // 2过电压
        Zdrive_InstabilityCurrent,  // 3电流不稳
        Zdrive_OverCurrent,         // 4过电流
        Zdrive_OverSpeed,           // 5超速
        Zdrive_ExcessiveR,          // 6电阻过大
        Zdrive_ExcessiveInductence, // 7电感过大
        Zdrive_LoseEncoder1,        // 8编码器错误
        Zdrive_PolesErr,            // 9极对数不匹配
        Zdrive_VKCalibrationErr,    // 10 KV校准失败
        Zdrive_ModeErr,             // 11模式不合法
        Zdrive_ParameterErr,        // 12参数错误
        Zdrive_Hot                  // 13过热
    } ZdriveErr;

    typedef enum
    {
        SES = 0x01,      // 有无感
        ENCODER,         // 编码器类型
        ENL,             // 线性补偿
        NodeID,          // 节点iD
        CAN_HZ,          // can频率
        HEARTBEAT,       // 心跳间隔
        Volta_LL,        // 电压下限
        Pos_UL,          // 位置上限 -1e4~1e4 r 协议单位
        Pos_LL,          // 位置下限 -1e4~1e4 r 协议单位
        Vel_Limit,       // 速度限制 绝对值 0.01~1ē4 rps 协议单位
        Poles_Num,       // 极对数
        CurLimit,        // 电流限制 
        CurrCAL,         // 校准电流
        Start_Mode,      // 启动模式 上电后自动进入的模式
        Answer_Mode,     // 响应模式 0:无反馈立即执行, 1:有反馈立即执行, 2:无反馈队列执行, 3:有反馈队列执行
        FilterCoeff,     // 滤波系数
        ToleranceCoeff,  // 容忍系数
        Pos_PID_P,       // 位置环kp 0~20
        Pos_PID_D,       // 位置环kd 0~20
        Vel_PID_P,       // 速度环kp 0~100
        Vel_PID_I,       // 速度环ki 0~100
        Acc_Acu,         // 加速度 rps² 协议单位
        Acc_Dec,         // 减速度 rps² 协议单位
        Pos_Vel_TimeGap, // 速度位置控制下的每帧间隔时间 s
        Mode = 0x1F,     // 模式
        Warning,         // 警告
        Err,             // 错误
        CurIn,           // 电流输入 A 协议单位
        VelIn,           // 速度输入 rps 协议单位
        PosIn,           // 位置输入 r 协议单位
        LIF,             // 队列控制
        Cur_M = 0x2B,    // 当前电流幅值 A 协议单位
        Vel = 0x2D,       // 当前转速 rps 协议单位
        Pur = 0x2E,       // 当前位置 r 协议单位
        PVT_Frame = 0x39, // PVT模式
        MIT_Frame = 0x38, // MIT模式
    } ZdriveCmd;

    typedef struct
    {
        float speed_rpm;   /* 速度(rpm) */
        float pos_deg;     /* 位置(deg) */
        float posIn_deg;   /* 当前位置(角度,deg,Pur 命令码) */
        float current_A;   /* 电流(A) */
        float accAcu_rps2; /* 加速度(rps²) */
        float accDec_rps2; /* 减速度(rps²) */
    } ZdriveValue;

    typedef struct
    {
        float velLimit_rpm;      /* 速度限制(rpm) */
        float curLimit_A;        /* 电流限制(A) */
        float posLimit_up_deg;   /* 位置限制(deg) */
        float posLimit_down_deg; /* 位置限制(deg) */
    } ZdriveLimit;

    typedef struct
    {
        float GearRatio;
        float ReductionRatio;
        uint8_t zdrive_id; /* 电机 ID,1..14 */
        float kpPos;       /* 位置环 P */
        float kdPos;       /* 位置环 D */
        float kpVel;       /* 速度环 P */
        float kiVel;       /* 速度环 I */
    } ZdriveParam;

    typedef struct
    {
        bool PVTflag;
        bool PVTinitflag;
        float deltaT;
        uint8_t answer_mode; /* 响应模式: 0:无反馈立即执行, 1:有反馈立即执行, 2:无反馈队列执行, 3:有反馈队列执行 */
    } ZdrivePVTParam;

    typedef struct
    {
        volatile bool Begin; /* 初始化完成标志:false 时 Func 跳过该电机,由任务层置 true */\
        volatile bool ZeroFlag; 
        ZdriveMode mode;     /* 目标模式,任务层写;Disable 即停止 */
        ZdriveMode modeRead; /* 驱动确认的当前模式,由 RX 更新 */
        ZdriveValue valSetNow;
        ZdriveValue valReal;
        ZdrivePVTParam pvtparam;
        ZdriveErr err;
        ZdriveValue valSetPre;
        ZdriveValue valPre;
        ZdriveParam param;
        ZdriveLimit limit;
    } Zdrive;

#if USE_ZMDR
    extern Zdrive Zmotor[USE_ZDRIVE_NUM];

    void ZdriveInit(void);
    void ZdriveFunc(void);
    void ZdriveReceive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data, uint8_t bus);
    void ZdriveDequeue(uint8_t bus);
    bool Zdrive_IsOurs(const FDCAN_RxHeaderTypeDef *Rxheader, uint8_t bus);

    void ZdriveSet(float data, uint8_t id, uint8_t set_code);
    void ZdriveAsk(uint8_t id, uint8_t ask_code);

    // 使用 PVT/ MIT 轨迹帧时,请注意:
    // 1. PVT/ MIT 帧不走 ZdriveSet,请直接调用 ZdriveSetPVT/ ZdriveSetMIT
    // 2. FUNC函数没有对 PVT/ MIT 帧做很好的适配,使用的时候需要自行调用set函数
    // 3. 函数的单位以协议为准,如速度为rps,位置为n.但是单位这一块还有问题,要使用请自行调整

    /** PVT 轨迹帧:一次下发速度+位置,8 字节双值,不走 ZdriveSet */
    void ZdriveSetPVT(float speed, float angle, uint8_t id);
    /** MIT 轨迹帧:一次下发速度+位置+电流+位置环/速度环 PID,8 字节多值,不走 ZdriveSet */
    void ZdriveSetMIT(float speed, float angle, float current, float pos_kp, float vel_kp, uint8_t id);
    /** 覆盖指定电机的 param,位置环/速度环 PID 哪一项改动就下发哪一项命令 */
    void ZdriveParamConfig(uint8_t id, ZdriveParam param);
#endif /* USE_ZMDR */

#ifdef __cplusplus
}
#endif

#endif /* ZDRIVE_H */
