/** @file mechanism.c
 *  @brief 机构状态、零位和三轴同步轨迹。
 */
#include "mechanism.h"
#include <math.h>
#include <string.h>

#define MECH_MAPPING_EPSILON 1e-6f
#define MECH_SECONDS_PER_MS 0.001f
#define MECH_TRAJECTORY_SPEED_FACTOR 1.875f
#define MECH_TRAJECTORY_ACCEL_FACTOR 6.0f

/* 正常流程：UNCALIBRATED --标零--> READY --使能--> HOLDING
 *           HOLDING --目标--> MOVING --轨迹结束--> HOLDING
 * 停止：有有效零位时回到 READY，否则回到 UNCALIBRATED。
 * 故障：使能期间反馈异常或实际越界进入 FAULT，清故障后必须重新标零。
 * state 是状态判断依据，原有布尔字段仅用于兼容输出和观测。
 */
static void Mechanism_EnterState(Mechanism_t *p_mechanism, Mechanism_State_t state)
{
    p_mechanism->state = state;
    p_mechanism->zero_valid = state == MECH_STATE_READY || state == MECH_STATE_HOLDING ||
                              state == MECH_STATE_MOVING;
    p_mechanism->moving = state == MECH_STATE_MOVING;
    p_mechanism->output.motors_enabled =
        state == MECH_STATE_HOLDING || state == MECH_STATE_MOVING;
}

static bool Mechanism_IsPositive(float value)
{
    return isfinite(value) && value > 0.0f;
}
static bool Mechanism_MappingValid(const Mechanism_Config_t *p_config)
{
    float determinant;
    if (p_config == NULL)
    {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        for (unsigned j = 0; j < 2; ++j)
        {
            if (!isfinite(p_config->wrist[i][j]))
            {
                return false;
            }
        }
    }
    determinant = p_config->wrist[0][0] * p_config->wrist[1][1] -
                  p_config->wrist[0][1] * p_config->wrist[1][0];
    return isfinite(determinant) && fabsf(determinant) > MECH_MAPPING_EPSILON;
}

bool Mechanism_ConfigValid(const Mechanism_Config_t *p_config)
{
    if (!Mechanism_MappingValid(p_config) || !isfinite(p_config->arm_motor_per_joint) ||
        fabsf(p_config->arm_motor_per_joint) < MECH_MAPPING_EPSILON ||
        p_config->feedback_timeout_ms == 0)
    {
        return false;
    }
    for (unsigned i = 0; i < MECH_AXIS_COUNT; ++i)
    {
        const Mechanism_AxisConfig_t *p_axis = &p_config->axis[i];
        if (!isfinite(p_axis->min_deg) || !isfinite(p_axis->max_deg) ||
            p_axis->min_deg >= p_axis->max_deg || p_axis->min_deg > 0 || p_axis->max_deg < 0 ||
            !Mechanism_IsPositive(p_axis->max_speed_deg_s) ||
            !Mechanism_IsPositive(p_axis->max_accel_deg_s2))
        {
            return false;
        }
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        if (!Mechanism_IsPositive(p_config->wrist_motor_speed_deg_s[i]) ||
            !Mechanism_IsPositive(p_config->wrist_motor_accel_deg_s2[i]))
        {
            return false;
        }
    }
    return true;
}

bool Mechanism_WristForward(const Mechanism_Config_t *p_config, float pitch, float roll,
                            float shaft[2])
{
    if (shaft == NULL || !Mechanism_MappingValid(p_config) || !isfinite(pitch) || !isfinite(roll))
    {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        shaft[i] = p_config->wrist[i][0] * pitch + p_config->wrist[i][1] * roll;
        if (!isfinite(shaft[i]))
        {
            return false;
        }
    }
    return true;
}

bool Mechanism_WristInverse(const Mechanism_Config_t *p_config, const float shaft[2], float *pitch,
                            float *roll)
{
    float determinant;
    if (shaft == NULL || pitch == NULL || roll == NULL || !Mechanism_MappingValid(p_config) ||
        !isfinite(shaft[0]) || !isfinite(shaft[1]))
    {
        return false;
    }
    determinant = p_config->wrist[0][0] * p_config->wrist[1][1] -
                  p_config->wrist[0][1] * p_config->wrist[1][0];
    *pitch = (shaft[0] * p_config->wrist[1][1] - p_config->wrist[0][1] * shaft[1]) / determinant;
    *roll = (p_config->wrist[0][0] * shaft[1] - shaft[0] * p_config->wrist[1][0]) / determinant;
    return isfinite(*pitch) && isfinite(*roll);
}

static Mechanism_Result_t Mechanism_FeedbackStatus(const Mechanism_t *p_mechanism,
                                                   const Mechanism_Feedback_t *p_feedback,
                                                   uint32_t now_ms)
{
    if (p_feedback->motor_error)
    {
        return MECH_MOTOR_ERROR;
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        if (!p_feedback->seen[i] ||
            now_ms - p_feedback->rx_ms[i] > p_mechanism->config.feedback_timeout_ms ||
            !isfinite(p_feedback->shaft_deg[i]))
        {
            return MECH_FEEDBACK_LOST;
        }
    }
    return MECH_OK;
}

static bool Mechanism_PoseValid(const Mechanism_t *p_mechanism, const Mechanism_Pose_t *p_pose)
{
    for (unsigned i = 0; i < MECH_AXIS_COUNT; ++i)
    {
        if (!isfinite(p_pose->axis[i]) || p_pose->axis[i] < p_mechanism->config.axis[i].min_deg ||
            p_pose->axis[i] > p_mechanism->config.axis[i].max_deg)
        {
            return false;
        }
    }
    return true;
}

static void Mechanism_Measure(Mechanism_t *p_mechanism, const Mechanism_Feedback_t *p_feedback)
{
    float shaft[2] = {p_feedback->shaft_deg[1] - p_mechanism->zero_shaft_deg[1],
                      p_feedback->shaft_deg[2] - p_mechanism->zero_shaft_deg[2]};
    p_mechanism->measured.axis[MECH_ARM] =
        (p_feedback->shaft_deg[0] - p_mechanism->zero_shaft_deg[0]) /
        p_mechanism->config.arm_motor_per_joint;
    Mechanism_WristInverse(&p_mechanism->config, shaft, &p_mechanism->measured.axis[MECH_PITCH],
                           &p_mechanism->measured.axis[MECH_ROLL]);
}

static void Mechanism_Fault(Mechanism_t *p_mechanism, Mechanism_Result_t reason)
{
    memset(&p_mechanism->output, 0, sizeof(p_mechanism->output));
    p_mechanism->fault = reason;
    Mechanism_EnterState(p_mechanism, MECH_STATE_FAULT);
}

void Mechanism_Init(Mechanism_t *p_mechanism, const Mechanism_Config_t *p_config, uint32_t now_ms)
{
    if (p_mechanism == NULL)
    {
        return;
    }
    memset(p_mechanism, 0, sizeof(*p_mechanism));
    if (p_config != NULL)
    {
        p_mechanism->config = *p_config;
    }
    p_mechanism->last_tick_ms = now_ms;
    Mechanism_EnterState(p_mechanism, MECH_STATE_UNCALIBRATED);
}

Mechanism_Result_t Mechanism_CaptureZero(Mechanism_t *p_mechanism,
                                         const Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    if (p_mechanism == NULL || p_feedback == NULL)
    {
        return MECH_BAD_STATE;
    }
    Mechanism_Result_t result;
    if (p_mechanism->state == MECH_STATE_HOLDING || p_mechanism->state == MECH_STATE_MOVING)
    {
        return MECH_BAD_STATE;
    }
    if (p_mechanism->state == MECH_STATE_FAULT)
    {
        return p_mechanism->fault;
    }
    if (!p_mechanism->config.motion_confirmed || !Mechanism_ConfigValid(&p_mechanism->config))
    {
        return MECH_BAD_CONFIG;
    }
    result = Mechanism_FeedbackStatus(p_mechanism, p_feedback, now_ms);
    if (result != MECH_OK)
    {
        return result;
    }
    memcpy(p_mechanism->zero_shaft_deg, p_feedback->shaft_deg, sizeof(p_mechanism->zero_shaft_deg));
    memset(&p_mechanism->measured, 0, sizeof(p_mechanism->measured));
    memset(&p_mechanism->setpoint, 0, sizeof(p_mechanism->setpoint));
    memset(&p_mechanism->target, 0, sizeof(p_mechanism->target));
    Mechanism_EnterState(p_mechanism, MECH_STATE_READY);
    return MECH_OK;
}

Mechanism_Result_t Mechanism_Enable(Mechanism_t *p_mechanism,
                                    const Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    if (p_mechanism == NULL || p_feedback == NULL)
    {
        return MECH_BAD_STATE;
    }
    Mechanism_Result_t result;
    if (p_mechanism->state == MECH_STATE_FAULT)
    {
        return p_mechanism->fault;
    }
    if (!p_mechanism->config.motion_confirmed || !Mechanism_ConfigValid(&p_mechanism->config))
    {
        return MECH_BAD_CONFIG;
    }
    if (p_mechanism->state == MECH_STATE_UNCALIBRATED)
    {
        return MECH_NO_ZERO;
    }
    result = Mechanism_FeedbackStatus(p_mechanism, p_feedback, now_ms);
    if (result != MECH_OK)
    {
        return result;
    }
    Mechanism_Measure(p_mechanism, p_feedback);
    if (!Mechanism_PoseValid(p_mechanism, &p_mechanism->measured))
    {
        return MECH_OUT_OF_RANGE;
    }
    if (p_mechanism->state == MECH_STATE_HOLDING || p_mechanism->state == MECH_STATE_MOVING)
    {
        return MECH_OK;
    }
    p_mechanism->setpoint = p_mechanism->measured;
    p_mechanism->target = p_mechanism->measured;
    p_mechanism->start = p_mechanism->measured;
    memcpy(p_mechanism->output.shaft_target_deg, p_feedback->shaft_deg,
           sizeof(p_mechanism->output.shaft_target_deg));
    memset(p_mechanism->output.shaft_speed_deg_s, 0, sizeof(p_mechanism->output.shaft_speed_deg_s));
    Mechanism_EnterState(p_mechanism, MECH_STATE_HOLDING);
    p_mechanism->last_tick_ms = now_ms;
    return MECH_OK;
}

/* 五次轨迹最大一阶导数为 1.875，二阶导数绝对值小于 6。 */
static float Mechanism_TrajectoryDuration(float delta, float speed, float accel)
{
    delta = fabsf(delta);
    return fmaxf(MECH_TRAJECTORY_SPEED_FACTOR * delta / speed,
                 sqrtf(MECH_TRAJECTORY_ACCEL_FACTOR * delta / accel));
}

Mechanism_Result_t Mechanism_SetTarget(Mechanism_t *p_mechanism, const Mechanism_Pose_t *p_pose,
                                       uint32_t now_ms)
{
    if (p_mechanism == NULL || p_pose == NULL)
    {
        return MECH_BAD_TARGET;
    }
    float delta[2], duration_s = 0.0f;
    if (p_mechanism->state != MECH_STATE_HOLDING && p_mechanism->state != MECH_STATE_MOVING)
    {
        return MECH_BAD_STATE;
    }
    if (!Mechanism_PoseValid(p_mechanism, p_pose))
    {
        return MECH_BAD_TARGET;
    }
    if (p_mechanism->state == MECH_STATE_MOVING)
    {
        return MECH_BUSY;
    }
    for (unsigned i = 0; i < MECH_AXIS_COUNT; ++i)
    {
        duration_s =
            fmaxf(duration_s,
                  Mechanism_TrajectoryDuration(p_pose->axis[i] - p_mechanism->setpoint.axis[i],
                                               p_mechanism->config.axis[i].max_speed_deg_s,
                                               p_mechanism->config.axis[i].max_accel_deg_s2));
    }
    if (!Mechanism_WristForward(
            &p_mechanism->config, p_pose->axis[MECH_PITCH] - p_mechanism->setpoint.axis[MECH_PITCH],
            p_pose->axis[MECH_ROLL] - p_mechanism->setpoint.axis[MECH_ROLL], delta))
    {
        return MECH_BAD_TARGET;
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        duration_s = fmaxf(duration_s, Mechanism_TrajectoryDuration(
                                           delta[i], p_mechanism->config.wrist_motor_speed_deg_s[i],
                                           p_mechanism->config.wrist_motor_accel_deg_s2[i]));
    }
    if (!isfinite(duration_s))
    {
        return MECH_BAD_TARGET;
    }
    p_mechanism->start = p_mechanism->setpoint;
    p_mechanism->target = *p_pose;
    p_mechanism->duration_s = duration_s;
    p_mechanism->elapsed_s = 0;
    Mechanism_EnterState(p_mechanism,
                         duration_s > 0 ? MECH_STATE_MOVING : MECH_STATE_HOLDING);
    p_mechanism->last_tick_ms = now_ms;
    return MECH_OK;
}

Mechanism_Result_t Mechanism_SetAir(Mechanism_t *p_mechanism, bool pump, Mechanism_Claw_t claw)
{
    if (p_mechanism == NULL)
    {
        return MECH_BAD_STATE;
    }
    if (p_mechanism->state == MECH_STATE_FAULT)
    {
        return p_mechanism->fault;
    }
    if (!p_mechanism->config.pneumatics_confirmed)
    {
        return MECH_BAD_CONFIG;
    }
    if (claw < MECH_CLAW_OFF || claw > MECH_CLAW_CLOSE)
    {
        return MECH_BAD_TARGET;
    }
    p_mechanism->output.pump_on = pump;
    p_mechanism->output.claw = claw;
    return MECH_OK;
}

void Mechanism_Stop(Mechanism_t *p_mechanism)
{
    if (p_mechanism == NULL)
    {
        return;
    }
    /* 正常停止保留零位；故障或反馈丢失后停止不会恢复零位。 */
    Mechanism_State_t next_state = MECH_STATE_UNCALIBRATED;
    if (p_mechanism->state == MECH_STATE_READY || p_mechanism->state == MECH_STATE_HOLDING ||
        p_mechanism->state == MECH_STATE_MOVING)
    {
        next_state = MECH_STATE_READY;
    }
    memset(&p_mechanism->output, 0, sizeof(p_mechanism->output));
    p_mechanism->fault = MECH_OK;
    Mechanism_EnterState(p_mechanism, next_state);
}

/* 三轴共用同一运动进度，起止速度和加速度均为零。 */
static void Mechanism_UpdateTrajectory(Mechanism_t *p_mechanism, float elapsed_s, float speed[3])
{
    float progress, position_scale, speed_scale;
    p_mechanism->elapsed_s = fminf(p_mechanism->elapsed_s + elapsed_s, p_mechanism->duration_s);
    progress = p_mechanism->elapsed_s / p_mechanism->duration_s;
    position_scale =
        progress * progress * progress * (10.0f + progress * (-15.0f + 6.0f * progress));
    speed_scale = 30.0f * progress * progress * (1.0f - progress) * (1.0f - progress) /
                  p_mechanism->duration_s;
    for (unsigned i = 0; i < MECH_AXIS_COUNT; ++i)
    {
        float delta_deg = p_mechanism->target.axis[i] - p_mechanism->start.axis[i];
        p_mechanism->setpoint.axis[i] = p_mechanism->start.axis[i] + delta_deg * position_scale;
        speed[i] = delta_deg * speed_scale;
    }
    if (p_mechanism->elapsed_s >= p_mechanism->duration_s)
    {
        Mechanism_EnterState(p_mechanism, MECH_STATE_HOLDING);
    }
}

/* 机构角度加上零位和传动映射，得到驱动层使用的电机输出轴目标。 */
static void Mechanism_UpdateMotorTargets(Mechanism_t *p_mechanism, const float speed[3])
{
    float wrist[2], wrist_speed[2];
    Mechanism_WristForward(&p_mechanism->config, p_mechanism->setpoint.axis[MECH_PITCH],
                           p_mechanism->setpoint.axis[MECH_ROLL], wrist);
    Mechanism_WristForward(&p_mechanism->config, speed[MECH_PITCH], speed[MECH_ROLL], wrist_speed);
    p_mechanism->output.shaft_target_deg[0] =
        p_mechanism->zero_shaft_deg[0] +
        p_mechanism->config.arm_motor_per_joint * p_mechanism->setpoint.axis[MECH_ARM];
    p_mechanism->output.shaft_speed_deg_s[0] =
        p_mechanism->config.arm_motor_per_joint * speed[MECH_ARM];
    for (unsigned i = 0; i < 2; ++i)
    {
        p_mechanism->output.shaft_target_deg[i + 1] = p_mechanism->zero_shaft_deg[i + 1] + wrist[i];
        p_mechanism->output.shaft_speed_deg_s[i + 1] = wrist_speed[i];
    }
}

void Mechanism_Tick(Mechanism_t *p_mechanism, const Mechanism_Feedback_t *p_feedback,
                    uint32_t now_ms)
{
    if (p_mechanism == NULL)
    {
        return;
    }
    if (p_feedback == NULL)
    {
        Mechanism_Fault(p_mechanism, MECH_FEEDBACK_LOST);
        return;
    }
    float elapsed_s = (float)(now_ms - p_mechanism->last_tick_ms) * MECH_SECONDS_PER_MS;
    float speed[3] = {0};
    Mechanism_Result_t result;
    p_mechanism->last_tick_ms = now_ms;

    switch (p_mechanism->state)
    {
    case MECH_STATE_UNCALIBRATED:
    case MECH_STATE_FAULT:
        /* 等待显式命令；反馈恢复不会自动标零或使能。 */
        return;

    case MECH_STATE_READY:
        if (Mechanism_FeedbackStatus(p_mechanism, p_feedback, now_ms) == MECH_OK)
        {
            Mechanism_Measure(p_mechanism, p_feedback);
        }
        else
        {
            /* 电机未使能时只作废零位，保留独立气路输出。 */
            Mechanism_EnterState(p_mechanism, MECH_STATE_UNCALIBRATED);
        }
        return;

    case MECH_STATE_HOLDING:
    case MECH_STATE_MOVING:
        result = Mechanism_FeedbackStatus(p_mechanism, p_feedback, now_ms);
        if (result != MECH_OK)
        {
            Mechanism_Fault(p_mechanism, result);
            return;
        }
        Mechanism_Measure(p_mechanism, p_feedback);
        if (!Mechanism_PoseValid(p_mechanism, &p_mechanism->measured))
        {
            Mechanism_Fault(p_mechanism, MECH_OUT_OF_RANGE);
            return;
        }
        if (p_mechanism->state == MECH_STATE_MOVING)
        {
            Mechanism_UpdateTrajectory(p_mechanism, elapsed_s, speed);
        }
        Mechanism_UpdateMotorTargets(p_mechanism, speed);
        return;

    default:
        Mechanism_Fault(p_mechanism, MECH_BAD_STATE);
        return;
    }
}
