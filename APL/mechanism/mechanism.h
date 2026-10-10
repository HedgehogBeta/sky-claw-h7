/** @file mechanism.h
 *  @brief 机械臂、差速腕部的零位、状态与同步运动。
 */
#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include <stdint.h>

#define MECH_FEEDBACK_TIMEOUT_MS 100U

typedef enum
{
    MECH_GO = 0,
    MECH_LEFT = 1,
    MECH_RIGHT = 2,
    MECH_MOTOR_COUNT = 3
} Mechanism_Motor_t;

typedef enum
{
    MECH_CLAW_OFF = 0,
    MECH_CLAW_OPEN = 1,
    MECH_CLAW_CLOSE = 2
} Mechanism_Claw_t;

typedef enum
{
    MECH_OK = 0,
    MECH_BAD_CONFIG = 1,
    MECH_BAD_STATE = 2,
    MECH_NO_ZERO = 3,
    MECH_FEEDBACK_LOST = 4,
    MECH_MOTOR_ERROR = 5,
    MECH_BAD_TARGET = 6,
    MECH_OUT_OF_RANGE = 7,
    MECH_BUSY = 8
} Mechanism_Result_t;

typedef enum
{
    MECH_STATE_UNCALIBRATED = 0, /* 失能，等待在机械零位标定 */
    MECH_STATE_READY = 1,       /* 零位有效，等待使能 */
    MECH_STATE_HOLDING = 2,     /* 使能，保持当前给定 */
    MECH_STATE_MOVING = 3,      /* 使能，执行同步轨迹 */
    MECH_STATE_FAULT = 4        /* 停止全部输出，等待停止命令 */
} Mechanism_State_t;

typedef struct
{
    float arm_deg;   /* 机械臂在平面内的转角 */
    float pitch_deg; /* 腕部相对机械臂的俯仰角 */
    float roll_deg;  /* 夹爪绕自身轴的旋转角 */
} Mechanism_Pose_t;

typedef struct
{
    float min_deg;
    float max_deg;
} Mechanism_Range_t;

/* 角度均为度；传动比仅描述电机减速器输出轴之后的机械传动。 */
typedef struct
{
    float arm_motor_ratio;     /* GO 输出轴角 / 机械臂角，带正负方向 */
    float pitch_motor_ratio;   /* 左 M2006 输出轴角 / 俯仰角，带正负方向 */
    float roll_motor_ratio;    /* 左 M2006 输出轴角 / 旋转角，带正负方向 */
    int right_motor_direction; /* 右电机与左电机的轴向约定：+1 或 -1 */
    Mechanism_Range_t arm_range;
    Mechanism_Range_t pitch_range;
    Mechanism_Range_t roll_range;
    float move_time_s;          /* 每次运动的最短时间，五次插值平滑起停 */
    float go_speed_limit_deg_s; /* GO 减速器输出轴速度上限 */
    int wrist_speed_limit_rpm;  /* 两台 M2006 减速器输出轴速度上限 */
} Mechanism_Config_t;

typedef struct
{
    float shaft_deg[MECH_MOTOR_COUNT];
    uint32_t rx_ms[MECH_MOTOR_COUNT]; /* 最近一帧有效反馈时间 */
    bool seen[MECH_MOTOR_COUNT];     /* 本次上电是否收到过有效反馈 */
    bool motor_error;               /* GO 驱动器错误 */
} Mechanism_Feedback_t;

typedef struct
{
    bool motors_enabled;
    float shaft_target_deg[MECH_MOTOR_COUNT];
    float go_speed_deg_s; /* GO 速度前馈；M2006 使用模板的位置闭环 */
    bool pump_on;
    Mechanism_Claw_t claw;
} Mechanism_Output_t;

typedef struct
{
    Mechanism_Config_t config;
    Mechanism_State_t state;  /* 状态的唯一依据 */
    Mechanism_Result_t fault; /* 故障原因，停止命令清除 */
    float zero_shaft_deg[MECH_MOTOR_COUNT];
    Mechanism_Pose_t measured;
    Mechanism_Pose_t setpoint;
    Mechanism_Pose_t target;
    Mechanism_Pose_t start;
    uint32_t last_tick_ms;
    float elapsed_s;
    float duration_s;
    Mechanism_Output_t output;
} Mechanism_t;

/** @brief 检查传动比、含零位的行程和运动时间、速度上限。 */
bool Mechanism_ConfigValid(const Mechanism_Config_t *p_config);
/** @brief 初始化为失能、未标零状态；时间单位为毫秒。 */
void Mechanism_Init(Mechanism_t *p_mechanism, const Mechanism_Config_t *p_config, uint32_t now_ms);
/** @brief 电机失能且机构位于约定零位时，记录三台电机的轴角。 */
Mechanism_Result_t Mechanism_CaptureZero(Mechanism_t *p_mechanism,
                                        const Mechanism_Feedback_t *p_feedback, uint32_t now_ms);
/** @brief 以当前实测姿态使能；重复使能保持原有轨迹。 */
Mechanism_Result_t Mechanism_Enable(Mechanism_t *p_mechanism,
                                   const Mechanism_Feedback_t *p_feedback, uint32_t now_ms);
/** @brief 设置三轴目标（度）；运动中返回 MECH_BUSY。 */
Mechanism_Result_t Mechanism_SetTarget(Mechanism_t *p_mechanism, const Mechanism_Pose_t *p_pose,
                                      uint32_t now_ms);
/** @brief 设置气路输出；硬件接线由应用层检查。 */
Mechanism_Result_t Mechanism_SetAir(Mechanism_t *p_mechanism, bool pump_on, Mechanism_Claw_t claw);
/** @brief 停止全部输出、清除故障；正常停止保留零位，故障后须重新标零。 */
void Mechanism_Stop(Mechanism_t *p_mechanism);
/** @brief 更新反馈、故障和轨迹；由应用层下发 output。 */
void Mechanism_Tick(Mechanism_t *p_mechanism, const Mechanism_Feedback_t *p_feedback,
                    uint32_t now_ms);

#endif /* MECHANISM_H */