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
    MECH_DISABLED,
    MECH_MANUAL,
    MECH_FAULT
} Mechanism_State_t;
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
    MECH_BUSY,
    MECH_LEASE_EXPIRED
} Mechanism_Result_t;

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
    uint32_t command_lease_ms;
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
    Mechanism_State_t state;
    Mechanism_Result_t fault;
    bool zero_valid; /* 本次上电的机械零位有效 */
    bool moving;     /* 轨迹尚未结束，与实际到位状态独立 */
    float zero_shaft_deg[3];
    Mechanism_Pose_t measured, setpoint, target, start;
    uint32_t last_tick_ms, last_command_ms;
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
Mechanism_Result_t Mechanism_SetAir(Mechanism_t *p_mechanism, bool pump_on, Mechanism_Claw_t claw,
                                    uint32_t now_ms);
/** @brief 停止电机和气路输出，保留故障锁存状态。 */
void Mechanism_Stop(Mechanism_t *p_mechanism);
/** @brief 反馈恢复后清除故障，进入失能状态并等待重新标定。 */
Mechanism_Result_t Mechanism_ClearFault(Mechanism_t *p_mechanism,
                                        const Mechanism_Feedback_t *p_feedback, uint32_t now_ms);
/** @brief 刷新手动控制租期，避免无人操作时持续驱动。 */
void Mechanism_KeepAlive(Mechanism_t *p_mechanism, uint32_t now_ms);
/** @brief 周期更新反馈、失效检查和轨迹，所有输出由调用者下发。 */
void Mechanism_Tick(Mechanism_t *p_mechanism, const Mechanism_Feedback_t *p_feedback,
                    uint32_t now_ms);

#endif /* MECHANISM_H */
