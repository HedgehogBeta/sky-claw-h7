/** @file mechanism.h
 *  @brief 可在主机验证的机构控制接口。
 */
#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MECH_ARM,
    MECH_PITCH,
    MECH_ROLL,
    MECH_AXIS_COUNT
} Mechanism_Axis_t;
typedef enum
{
    MECH_CLAW_OFF,
    MECH_CLAW_OPEN,
    MECH_CLAW_CLOSE
} Mechanism_Claw_t;
typedef enum
{
    MECH_OK,
    MECH_BAD_CONFIG,
    MECH_BAD_STATE,
    MECH_NO_ZERO,
    MECH_FEEDBACK_LOST,
    MECH_MOTOR_ERROR,
    MECH_BAD_TARGET,
    MECH_OUT_OF_RANGE,
    MECH_BUSY
} Mechanism_Result_t;

/* 运动状态；气路可在电机失能时单独工作，不增加运动状态。 */
typedef enum
{
    MECH_STATE_UNCALIBRATED = 0, /* 电机失能，等待标零 */
    MECH_STATE_READY = 1,       /* 零位有效，等待使能 */
    MECH_STATE_HOLDING = 2,     /* 电机使能，保持给定姿态 */
    MECH_STATE_MOVING = 3,      /* 电机使能，执行同步轨迹 */
    MECH_STATE_FAULT = 4        /* 全部输出停止，等待停止命令清除故障 */
} Mechanism_State_t;

/* 应用角度和电机减速器输出轴角度均使用度，内部减速比由电机驱动处理。 */
typedef struct
{
    float axis[MECH_AXIS_COUNT];
} Mechanism_Pose_t;
typedef struct
{
    float min_deg, max_deg, max_speed_deg_s, max_accel_deg_s2;
} Mechanism_AxisConfig_t;
typedef struct
{
    bool motion_confirmed;     /* 机械参数与电机增益已确认 */
    bool pneumatics_confirmed; /* 气路接线与失能状态已确认 */
    float arm_motor_per_joint; /* GO 输出轴角 / 机械臂关节角，含方向 */
    float wrist[2][2];         /* [左/右输出轴][俯仰/旋转]，含方向和外部传动比 */
    Mechanism_AxisConfig_t axis[MECH_AXIS_COUNT];
    float wrist_motor_speed_deg_s[2];
    float wrist_motor_accel_deg_s2[2];
    uint32_t feedback_timeout_ms;
} Mechanism_Config_t;
typedef struct
{
    float shaft_deg[3]; /* GO、左 M2006、右 M2006 */
    uint32_t rx_ms[3];
    bool seen[3];     /* 各电机至少收到一帧有效反馈 */
    bool motor_error; /* 驱动器上报错误 */
} Mechanism_Feedback_t;
typedef struct
{
    bool motors_enabled;
    float shaft_target_deg[3];
    float shaft_speed_deg_s[3];
    bool pump_on;
    Mechanism_Claw_t claw;
} Mechanism_Output_t;
typedef struct
{
    Mechanism_Config_t config;
    Mechanism_State_t state;  /* 状态机的当前状态，仅由机构接口更新 */
    Mechanism_Result_t fault; /* FAULT 状态的停止原因，停止命令清除 */
    /* 以下观测字段由 state 同步，保留原调用和 Watch 用法，请勿直接修改。 */
    bool zero_valid; /* 本次上电的机械零位有效 */
    bool moving;     /* 轨迹尚未结束，与实际到位状态独立 */
    float zero_shaft_deg[3];
    Mechanism_Pose_t measured, setpoint, target, start;
    uint32_t last_tick_ms;
    float elapsed_s, duration_s;
    Mechanism_Output_t output;
} Mechanism_t;

/** @brief 校验映射、限位及速度/加速度配置，零位必须位于允许范围内。 */
bool Mechanism_ConfigValid(const Mechanism_Config_t *p_config);
/** @brief 由俯仰/旋转角计算左右减速器输出轴角，单位均为度。 */
bool Mechanism_WristForward(const Mechanism_Config_t *p_config, float pitch, float roll,
                            float shaft[2]);
/** @brief 由左右输出轴角还原俯仰/旋转角，单位均为度。 */
bool Mechanism_WristInverse(const Mechanism_Config_t *p_config, const float shaft[2], float *pitch,
                            float *roll);
/** @brief 初始化为失能状态，时间单位为毫秒。 */
void Mechanism_Init(Mechanism_t *p_mechanism, const Mechanism_Config_t *p_config, uint32_t now_ms);
/** @brief 失能且机构位于约定零位时，记录三台电机的当前轴角。 */
Mechanism_Result_t Mechanism_CaptureZero(Mechanism_t *p_mechanism,
                                         const Mechanism_Feedback_t *p_feedback, uint32_t now_ms);
/** @brief 配置、零位和反馈有效时使能，以当前姿态作为初始目标。 */
Mechanism_Result_t Mechanism_Enable(Mechanism_t *p_mechanism,
                                    const Mechanism_Feedback_t *p_feedback, uint32_t now_ms);
/** @brief 设置三轴目标（度）；轨迹执行中返回 MECH_BUSY。 */
Mechanism_Result_t Mechanism_SetTarget(Mechanism_t *p_mechanism, const Mechanism_Pose_t *p_pose,
                                       uint32_t now_ms);
/** @brief 按已确认的气路配置设置泵/夹爪，电机可保持失能。 */
Mechanism_Result_t Mechanism_SetAir(Mechanism_t *p_mechanism, bool pump_on, Mechanism_Claw_t claw);
/** @brief 停止电机和气路输出，清除故障记录；故障后的零位仍需重新标定。 */
void Mechanism_Stop(Mechanism_t *p_mechanism);
/** @brief 周期更新反馈、失效检查和轨迹，所有输出由调用者下发。 */
void Mechanism_Tick(Mechanism_t *p_mechanism, const Mechanism_Feedback_t *p_feedback,
                    uint32_t now_ms);

#endif /* MECHANISM_H */
