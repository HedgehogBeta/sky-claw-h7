/** @file mechanism_app.h
 *  @brief H7 机构任务、中断接入与 Keil Watch 调试入口。
 */
#ifndef MECHANISM_APP_H
#define MECHANISM_APP_H

#include "mechanism.h"
#include "main.h"

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
    float angle_deg;   /* 减速器输出轴角度 */
    float speed_rpm;   /* 减速器输出轴速度 */
    float current_a;
    uint32_t age_ms;   /* 收到首帧后有效 */
    bool online;
} Mechanism_DebugDji_t;

typedef struct
{
    float angle_deg;   /* 减速器输出轴角度 */
    float speed_deg_s;
    float torque_nm;
    int temperature_c;
    uint32_t error;    /* GO 驱动器错误码，0 为正常 */
    uint32_t age_ms;   /* 收到首帧后有效 */
    bool online;
} Mechanism_DebugGo_t;

/* 启用机构控制后，参数先写、command 最后写；消费后 command=0。
 * MECH_CONTROL_ENABLE=0 时只观测反馈，控制命令返回 MECH_BAD_STATE。
 */
typedef struct
{
    uint32_t command;
    Mechanism_Pose_t pose;   /* SET_POSE 的三轴目标，deg */
    uint32_t pump_on;       /* SET_AIR 的泵开关：0/1 */
    uint32_t claw;          /* SET_AIR：0 失能、1 张开、2 夹紧 */
    Mechanism_Result_t result;

    bool motion_ready;     /* 机构控制已启用且机械参数、增益有效 */
    bool air_ready;        /* 机构控制已启用且气路配置有效 */
    Mechanism_State_t state;
    Mechanism_Result_t fault;
    Mechanism_Pose_t measured; /* READY/HOLDING/MOVING 状态下有效 */
    Mechanism_Pose_t setpoint;
    Mechanism_Pose_t target;
    Mechanism_Output_t output;
    Mechanism_DebugGo_t go;
    Mechanism_DebugDji_t left;
    Mechanism_DebugDji_t right;
} Mechanism_Debug_t;

extern volatile Mechanism_Debug_t g_mechanism_debug;

/** @brief MX_FREERTOS_Init 中创建机构任务。 */
void Mechanism_TaskCreate(void);
/** @brief TIM2 每毫秒调用，按 5 ms 周期唤醒机构任务。 */
void Mechanism_TimerCallback(void);
/** @brief 在 DJI 驱动接收之后记录有效反馈时间，不修改驱动接口。 */
void Mechanism_DjiFeedbackCallback(const FDCAN_RxHeaderTypeDef *p_header);
/** @brief UART7 接收适配：调用模板驱动并记录新有效 GO 反馈。 */
void Mechanism_GoReceive(UART_HandleTypeDef *p_uart);

#endif /* MECHANISM_APP_H */
