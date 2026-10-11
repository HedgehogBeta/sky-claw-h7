/** @file mechanism.c
 *  @brief 三轴同步五次轨迹、差速映射、标零和故障停机。
 */
#include "mechanism.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MECH_RATIO_EPSILON 1e-6f
#define MECH_QUINTIC_MAX_SPEED 1.875f /* 归一化五次曲线的一阶导数峰值 */
#define MECH_DEG_S_PER_RPM 6.0f

static bool Mechanism_HasZero(const Mechanism_t *p_mechanism)
{
    return p_mechanism->state == MECH_STATE_READY ||
           p_mechanism->state == MECH_STATE_HOLDING ||
           p_mechanism->state == MECH_STATE_MOVING;
}

static void Mechanism_EnterState(Mechanism_t *p_mechanism, Mechanism_State_t state)
{
    p_mechanism->state = state;
    p_mechanism->output.motors_enabled =
        state == MECH_STATE_HOLDING || state == MECH_STATE_MOVING;
}

static bool Mechanism_RangeValid(Mechanism_Range_t range)
{
    return isfinite(range.min_deg) && isfinite(range.max_deg) &&
           range.min_deg < range.max_deg && range.min_deg <= 0.0f && range.max_deg >= 0.0f;
}

bool Mechanism_ConfigValid(const Mechanism_Config_t *p_config)
{
    return p_config != NULL &&
           (!p_config->arm_enabled ||
            (isfinite(p_config->arm_motor_ratio) && fabsf(p_config->arm_motor_ratio) > MECH_RATIO_EPSILON &&
             Mechanism_RangeValid(p_config->arm_range) &&
             isfinite(p_config->go_speed_limit_deg_s) && p_config->go_speed_limit_deg_s > 0.0f)) &&
           isfinite(p_config->pitch_motor_ratio) && fabsf(p_config->pitch_motor_ratio) > MECH_RATIO_EPSILON &&
           isfinite(p_config->roll_motor_ratio) && fabsf(p_config->roll_motor_ratio) > MECH_RATIO_EPSILON &&
           (p_config->right_motor_direction == 1 || p_config->right_motor_direction == -1) &&
           Mechanism_RangeValid(p_config->pitch_range) &&
           Mechanism_RangeValid(p_config->roll_range) &&
           isfinite(p_config->move_time_s) && p_config->move_time_s > 0.0f &&
           p_config->wrist_speed_limit_rpm > 0 && p_config->wrist_speed_limit_rpm <= INT16_MAX;
}

static bool Mechanism_InRange(float angle_deg, Mechanism_Range_t range)
{
    return isfinite(angle_deg) && angle_deg >= range.min_deg && angle_deg <= range.max_deg;
}

static bool Mechanism_PoseValid(const Mechanism_Config_t *p_config,
                                const Mechanism_Pose_t *p_pose)
{
    return (p_config->arm_enabled ? Mechanism_InRange(p_pose->arm_deg, p_config->arm_range) :
                                   p_pose->arm_deg == 0.0f) &&
           Mechanism_InRange(p_pose->pitch_deg, p_config->pitch_range) &&
           Mechanism_InRange(p_pose->roll_deg, p_config->roll_range);
}

/* 同向分量驱动俯仰，反向分量驱动旋转；右电机方向先归一化。 */
static void Mechanism_ToMotorAngles(const Mechanism_Config_t *p_config,
                                    const Mechanism_Pose_t *p_pose, float shaft_deg[MECH_MOTOR_COUNT])
{
    float pitch = p_config->pitch_motor_ratio * p_pose->pitch_deg;
    float roll = p_config->roll_motor_ratio * p_pose->roll_deg;
    shaft_deg[MECH_GO] = p_config->arm_enabled ? p_config->arm_motor_ratio * p_pose->arm_deg : 0.0f;
    shaft_deg[MECH_LEFT] = pitch + roll;
    shaft_deg[MECH_RIGHT] = (float)p_config->right_motor_direction * (pitch - roll);
}

static void Mechanism_Measure(Mechanism_t *p_mechanism, const Mechanism_Feedback_t *p_feedback)
{
    const Mechanism_Config_t *p_config = &p_mechanism->config;
    float left = p_feedback->shaft_deg[MECH_LEFT] - p_mechanism->zero_shaft_deg[MECH_LEFT];
    float right = (p_feedback->shaft_deg[MECH_RIGHT] - p_mechanism->zero_shaft_deg[MECH_RIGHT]) *
                  (float)p_config->right_motor_direction;
    p_mechanism->measured.arm_deg = p_config->arm_enabled ?
        (p_feedback->shaft_deg[MECH_GO] - p_mechanism->zero_shaft_deg[MECH_GO]) / p_config->arm_motor_ratio :
        0.0f;
    p_mechanism->measured.pitch_deg = (0.5f * left + 0.5f * right) / p_config->pitch_motor_ratio;
    p_mechanism->measured.roll_deg = (0.5f * left - 0.5f * right) / p_config->roll_motor_ratio;
}

static Mechanism_Result_t Mechanism_FeedbackStatus(const Mechanism_Config_t *p_config,
                                                   const Mechanism_Feedback_t *p_feedback,
                                                   uint32_t now_ms)
{
    if (p_feedback == NULL)
    {
        return MECH_FEEDBACK_LOST;
    }
    if (p_config->arm_enabled && p_feedback->motor_error)
    {
        return MECH_MOTOR_ERROR;
    }
    unsigned first_motor = p_config->arm_enabled ? MECH_GO : MECH_LEFT;
    for (unsigned i = first_motor; i < MECH_MOTOR_COUNT; ++i)
    {
        if (!p_feedback->seen[i] || now_ms - p_feedback->rx_ms[i] > MECH_FEEDBACK_TIMEOUT_MS ||
            !isfinite(p_feedback->shaft_deg[i]))
        {
            return MECH_FEEDBACK_LOST;
        }
    }
    return MECH_OK;
}

static void Mechanism_Fault(Mechanism_t *p_mechanism, Mechanism_Result_t reason)
{
    memset(&p_mechanism->output, 0, sizeof(p_mechanism->output));
    p_mechanism->fault = reason;
    Mechanism_EnterState(p_mechanism, MECH_STATE_FAULT);
}

void Mechanism_Init(Mechanism_t *p_mechanism, const Mechanism_Config_t *p_config, uint32_t now_ms)
{
    memset(p_mechanism, 0, sizeof(*p_mechanism));
    p_mechanism->config = *p_config;
    p_mechanism->last_tick_ms = now_ms;
    Mechanism_EnterState(p_mechanism, MECH_STATE_UNCALIBRATED);
}

Mechanism_Result_t Mechanism_CaptureZero(Mechanism_t *p_mechanism,
                                        const Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    if (p_mechanism->state == MECH_STATE_FAULT)
    {
        return p_mechanism->fault;
    }
    if (p_mechanism->output.motors_enabled)
    {
        return MECH_BAD_STATE;
    }
    if (!Mechanism_ConfigValid(&p_mechanism->config))
    {
        return MECH_BAD_CONFIG;
    }
    Mechanism_Result_t result = Mechanism_FeedbackStatus(&p_mechanism->config, p_feedback, now_ms);
    if (result != MECH_OK)
    {
        return result;
    }
    memset(p_mechanism->zero_shaft_deg, 0, sizeof(p_mechanism->zero_shaft_deg));
    unsigned first_motor = p_mechanism->config.arm_enabled ? MECH_GO : MECH_LEFT;
    for (unsigned i = first_motor; i < MECH_MOTOR_COUNT; ++i)
    {
        p_mechanism->zero_shaft_deg[i] = p_feedback->shaft_deg[i];
    }
    p_mechanism->measured = (Mechanism_Pose_t){0};
    p_mechanism->setpoint = p_mechanism->measured;
    p_mechanism->target = p_mechanism->measured;
    Mechanism_EnterState(p_mechanism, MECH_STATE_READY);
    return MECH_OK;
}

Mechanism_Result_t Mechanism_Enable(Mechanism_t *p_mechanism,
                                   const Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    if (p_mechanism->state == MECH_STATE_FAULT)
    {
        return p_mechanism->fault;
    }
    if (!Mechanism_ConfigValid(&p_mechanism->config))
    {
        return MECH_BAD_CONFIG;
    }
    if (!Mechanism_HasZero(p_mechanism))
    {
        return MECH_NO_ZERO;
    }
    Mechanism_Result_t result = Mechanism_FeedbackStatus(&p_mechanism->config, p_feedback, now_ms);
    if (result != MECH_OK)
    {
        if (p_mechanism->output.motors_enabled)
        {
            Mechanism_Fault(p_mechanism, result);
        }
        else
        {
            Mechanism_EnterState(p_mechanism, MECH_STATE_UNCALIBRATED);
        }
        return result;
    }
    Mechanism_Measure(p_mechanism, p_feedback);
    if (!Mechanism_PoseValid(&p_mechanism->config, &p_mechanism->measured))
    {
        if (p_mechanism->output.motors_enabled)
        {
            Mechanism_Fault(p_mechanism, MECH_OUT_OF_RANGE);
        }
        return MECH_OUT_OF_RANGE;
    }
    if (!p_mechanism->output.motors_enabled)
    {
        p_mechanism->setpoint = p_mechanism->measured;
        p_mechanism->target = p_mechanism->measured;
        memset(p_mechanism->output.shaft_target_deg, 0, sizeof(p_mechanism->output.shaft_target_deg));
        unsigned first_motor = p_mechanism->config.arm_enabled ? MECH_GO : MECH_LEFT;
        for (unsigned i = first_motor; i < MECH_MOTOR_COUNT; ++i)
        {
            p_mechanism->output.shaft_target_deg[i] = p_feedback->shaft_deg[i];
        }
        p_mechanism->output.go_speed_deg_s = 0.0f;
        p_mechanism->output.dji_speed_rpm[0] = 0.0f;
        p_mechanism->output.dji_speed_rpm[1] = 0.0f;
        p_mechanism->last_tick_ms = now_ms;
        Mechanism_EnterState(p_mechanism, MECH_STATE_HOLDING);
    }
    return MECH_OK;
}

Mechanism_Result_t Mechanism_SetTarget(Mechanism_t *p_mechanism, const Mechanism_Pose_t *p_pose,
                                      uint32_t now_ms)
{
    if (!p_mechanism->output.motors_enabled)
    {
        return MECH_BAD_STATE;
    }
    if (p_pose == NULL || !Mechanism_PoseValid(&p_mechanism->config, p_pose))
    {
        return MECH_BAD_TARGET;
    }
    if (p_mechanism->state == MECH_STATE_MOVING)
    {
        return MECH_BUSY;
    }
    Mechanism_Pose_t delta = {
        p_pose->arm_deg - p_mechanism->setpoint.arm_deg,
        p_pose->pitch_deg - p_mechanism->setpoint.pitch_deg,
        p_pose->roll_deg - p_mechanism->setpoint.roll_deg};
    float shaft_delta[MECH_MOTOR_COUNT];
    float target_shaft[MECH_MOTOR_COUNT];
    Mechanism_ToMotorAngles(&p_mechanism->config, &delta, shaft_delta);
    Mechanism_ToMotorAngles(&p_mechanism->config, p_pose, target_shaft);
    for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
    {
        if (!isfinite(shaft_delta[i]) || !isfinite(target_shaft[i] + p_mechanism->zero_shaft_deg[i]))
        {
            return MECH_BAD_TARGET;
        }
    }
    /* 按电机轴的实际合成位移限速，俯仰与旋转同时运动也不会叠加超速。 */
    float duration_s = p_mechanism->config.move_time_s;
    if (p_mechanism->config.arm_enabled)
    {
        duration_s = fmaxf(duration_s, MECH_QUINTIC_MAX_SPEED * fabsf(shaft_delta[MECH_GO]) /
                                      p_mechanism->config.go_speed_limit_deg_s);
    }
    float wrist_speed_deg_s = MECH_DEG_S_PER_RPM * (float)p_mechanism->config.wrist_speed_limit_rpm;
    duration_s = fmaxf(duration_s, MECH_QUINTIC_MAX_SPEED *
                                  fmaxf(fabsf(shaft_delta[MECH_LEFT]), fabsf(shaft_delta[MECH_RIGHT])) /
                                  wrist_speed_deg_s);
    if (!isfinite(duration_s))
    {
        return MECH_BAD_TARGET;
    }
    p_mechanism->start = p_mechanism->setpoint;
    p_mechanism->target = *p_pose;
    p_mechanism->elapsed_s = 0.0f;
    p_mechanism->duration_s = duration_s;
    p_mechanism->output.go_speed_deg_s = 0.0f;
    p_mechanism->output.dji_speed_rpm[0] = 0.0f;
    p_mechanism->output.dji_speed_rpm[1] = 0.0f;
    p_mechanism->last_tick_ms = now_ms;
    if (delta.arm_deg != 0.0f || delta.pitch_deg != 0.0f || delta.roll_deg != 0.0f)
    {
        Mechanism_EnterState(p_mechanism, MECH_STATE_MOVING);
    }
    return MECH_OK;
}

Mechanism_Result_t Mechanism_SetAir(Mechanism_t *p_mechanism, bool pump_on, Mechanism_Claw_t claw)
{
    if (p_mechanism->state == MECH_STATE_FAULT)
    {
        return p_mechanism->fault;
    }
    if (claw < MECH_CLAW_OFF || claw > MECH_CLAW_CLOSE)
    {
        return MECH_BAD_TARGET;
    }
    p_mechanism->output.pump_on = pump_on;
    p_mechanism->output.claw = claw;
    return MECH_OK;
}

void Mechanism_Stop(Mechanism_t *p_mechanism)
{
    Mechanism_State_t state = Mechanism_HasZero(p_mechanism) ? MECH_STATE_READY : MECH_STATE_UNCALIBRATED;
    memset(&p_mechanism->output, 0, sizeof(p_mechanism->output));
    p_mechanism->fault = MECH_OK;
    Mechanism_EnterState(p_mechanism, state);
}

/* s(t)=10t³-15t⁴+6t⁵，三轴共用进度，起止速度与加速度均为零。 */
static void Mechanism_UpdateTrajectory(Mechanism_t *p_mechanism, float elapsed_s)
{
    p_mechanism->elapsed_s = fminf(p_mechanism->elapsed_s + elapsed_s, p_mechanism->duration_s);
    if (p_mechanism->elapsed_s >= p_mechanism->duration_s)
    {
        p_mechanism->setpoint = p_mechanism->target;
        p_mechanism->output.go_speed_deg_s = 0.0f;
        p_mechanism->output.dji_speed_rpm[0] = 0.0f;
        p_mechanism->output.dji_speed_rpm[1] = 0.0f;
        Mechanism_EnterState(p_mechanism, MECH_STATE_HOLDING);
        return;
    }
    float t = p_mechanism->elapsed_s / p_mechanism->duration_s;
    float position_scale = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
    float speed_scale = 30.0f * t * t * (1.0f - t) * (1.0f - t) / p_mechanism->duration_s;
    p_mechanism->setpoint.arm_deg = p_mechanism->start.arm_deg +
        (p_mechanism->target.arm_deg - p_mechanism->start.arm_deg) * position_scale;
    p_mechanism->setpoint.pitch_deg = p_mechanism->start.pitch_deg +
        (p_mechanism->target.pitch_deg - p_mechanism->start.pitch_deg) * position_scale;
    p_mechanism->setpoint.roll_deg = p_mechanism->start.roll_deg +
        (p_mechanism->target.roll_deg - p_mechanism->start.roll_deg) * position_scale;
    /* 导数与位置使用同一差速映射；前馈使电机无需等待位置误差才开始跟随。 */
    Mechanism_Pose_t velocity = {
        (p_mechanism->target.arm_deg - p_mechanism->start.arm_deg) * speed_scale,
        (p_mechanism->target.pitch_deg - p_mechanism->start.pitch_deg) * speed_scale,
        (p_mechanism->target.roll_deg - p_mechanism->start.roll_deg) * speed_scale};
    float shaft_speed_deg_s[MECH_MOTOR_COUNT];
    Mechanism_ToMotorAngles(&p_mechanism->config, &velocity, shaft_speed_deg_s);
    p_mechanism->output.go_speed_deg_s = shaft_speed_deg_s[MECH_GO];
    p_mechanism->output.dji_speed_rpm[0] = shaft_speed_deg_s[MECH_LEFT] / MECH_DEG_S_PER_RPM;
    p_mechanism->output.dji_speed_rpm[1] = shaft_speed_deg_s[MECH_RIGHT] / MECH_DEG_S_PER_RPM;
}

void Mechanism_Tick(Mechanism_t *p_mechanism, const Mechanism_Feedback_t *p_feedback,
                    uint32_t now_ms)
{
    float elapsed_s = (float)(now_ms - p_mechanism->last_tick_ms) * 0.001f;
    p_mechanism->last_tick_ms = now_ms;
    if (!Mechanism_HasZero(p_mechanism))
    {
        return;
    }
    Mechanism_Result_t result = Mechanism_FeedbackStatus(&p_mechanism->config, p_feedback, now_ms);
    if (result != MECH_OK)
    {
        if (p_mechanism->output.motors_enabled)
        {
            Mechanism_Fault(p_mechanism, result);
        }
        else
        {
            Mechanism_EnterState(p_mechanism, MECH_STATE_UNCALIBRATED);
        }
        return;
    }
    Mechanism_Measure(p_mechanism, p_feedback);
    if (!p_mechanism->output.motors_enabled)
    {
        return;
    }
    if (!Mechanism_PoseValid(&p_mechanism->config, &p_mechanism->measured))
    {
        Mechanism_Fault(p_mechanism, MECH_OUT_OF_RANGE);
        return;
    }
    if (p_mechanism->state == MECH_STATE_MOVING)
    {
        Mechanism_UpdateTrajectory(p_mechanism, elapsed_s);
    }
    Mechanism_ToMotorAngles(&p_mechanism->config, &p_mechanism->setpoint,
                            p_mechanism->output.shaft_target_deg);
    for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
    {
        p_mechanism->output.shaft_target_deg[i] += p_mechanism->zero_shaft_deg[i];
    }
}
