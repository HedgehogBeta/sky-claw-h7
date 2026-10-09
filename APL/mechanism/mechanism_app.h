/** @file mechanism_app.h
 *  @brief H7 机构任务和调试入口。
 */
#ifndef MECHANISM_APP_H
#define MECHANISM_APP_H
#include "mechanism.h"

typedef enum
{
    MECH_CMD_NONE = 0,
    MECH_CMD_STOP = 1,
    MECH_CMD_CAPTURE_ZERO = 2,
    MECH_CMD_ENABLE = 3,
    MECH_CMD_SET_POSE = 4,
    MECH_CMD_SET_AIR = 5
} Mechanism_Command_t;

typedef struct
{
    float arm_deg;   /* 机械臂角度，deg */
    float pitch_deg; /* 腕部相对俯仰角，deg */
    float roll_deg;  /* 腕部旋转角，deg */
} Mechanism_DebugPose_t;

typedef struct
{
    float angle_deg;       /* 减速器输出轴角度，标零前也可读，deg */
    float speed_rpm;       /* 减速器输出轴速度，rpm */
    float current_a;       /* 反馈电流，A */
    int32_t temperature_c; /* 温度，℃；是否有效取决于驱动器反馈 */
    uint32_t last_rx_ms;   /* 最近有效反馈的 HAL tick */
    uint32_t age_ms;       /* 距最近有效反馈的毫秒数，received 后有效 */
    bool received;        /* 至少收到过一帧有效反馈 */
    bool online;          /* 已收到反馈，且未超过反馈超时 */
} Mechanism_DebugDji_t;

typedef struct
{
    float angle_deg;       /* 减速器输出轴角度，标零前也可读，deg */
    float speed_deg_s;     /* 减速器输出轴速度，deg/s */
    float torque_nm;       /* 反馈力矩，N·m */
    int32_t temperature_c; /* 温度，℃ */
    uint32_t error;        /* GO 驱动器错误码，0 为无错误 */
    uint32_t bad_msg;      /* 包头/CRC 错误计数 */
    uint32_t last_rx_ms;   /* 最近有效反馈的 HAL tick */
    uint32_t age_ms;       /* 距最近有效反馈的毫秒数，received 后有效 */
    bool received;        /* 至少收到过一帧有效反馈 */
    bool online;          /* 已收到反馈，且未超过反馈超时 */
} Mechanism_DebugGo_t;

/* 调试器先写参数，最后写 command；每条命令消费后 command 回到 NONE。 */
typedef struct
{
    /* 可写命令区：先写角度或气路参数，最后写 command。 */
    uint32_t command;                  /* Mechanism_Command_t，0 为无命令 */
    float arm_deg, pitch_deg, roll_deg; /* 目标关节角，deg */
    uint32_t pump_on, claw;            /* 泵 0/1；夹爪 0 失能、1 张开、2 夹紧 */

    /* 观测区：由机构任务每 5 ms 更新，Watch 中只读。 */
    uint32_t result;                 /* 最近一条命令的 Mechanism_Result_t */
    uint32_t completed;              /* 已消费命令数 */
    bool motion_ready;               /* 运动参数、映射及增益配置有效 */
    bool air_ready;                  /* 气路配置与初始化有效 */
    Mechanism_State_t state;         /* 未标零 / 待使能 / 保持 / 运动 / 故障 */
    bool zero_valid;                 /* 当前零位有效 */
    bool moving;                     /* 轨迹执行中，不代表实际到位 */
    Mechanism_Result_t fault;        /* 停机原因；停止命令清除 */
    Mechanism_DebugPose_t measured;  /* 标零后的三轴实际角 */
    Mechanism_DebugPose_t setpoint;  /* 轨迹当前给定角 */
    Mechanism_DebugPose_t target;    /* 本段轨迹最终目标角 */
    Mechanism_Output_t output;       /* 实际下发的电机与气路命令 */
    Mechanism_DebugGo_t go;          /* GO 原始反馈，角度/速度已换算为度 */
    Mechanism_DebugDji_t left;       /* 左 M2006 反馈 */
    Mechanism_DebugDji_t right;      /* 右 M2006 反馈 */
} Mechanism_Debug_t;

extern volatile Mechanism_Debug_t g_mechanism_debug;
/** @brief 创建机构任务，供 MX_FREERTOS_Init 调用。 */
void Mechanism_TaskCreate(void);
/** @brief TIM2 每毫秒调用，按机构周期发出任务标志。 */
void Mechanism_TimerCallback(void);

#endif
