/** @file test_mechanism.c
 *  @brief 差速方向、同步限速、标零、停机恢复和反馈时效的主机验证。
 */
#include "mechanism.h"
#include "mechanism_config.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static Mechanism_Config_t Test_Config(void)
{
    const Mechanism_Config_t config = {
        .arm_enabled = true,
        .arm_motor_ratio = -2.0f,
        .pitch_motor_ratio = 3.0f,
        .roll_motor_ratio = -4.0f,
        .right_motor_direction = -1,
        .arm_range = {-90.0f, 90.0f},
        .pitch_range = {-90.0f, 90.0f},
        .roll_range = {-180.0f, 180.0f},
        .move_time_s = 0.2f,
        .go_speed_limit_deg_s = 60.0f,
        .wrist_speed_limit_rpm = 10};
    return config;
}

static void Test_Refresh(Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
    {
        p_feedback->rx_ms[i] = now_ms;
        p_feedback->seen[i] = true;
    }
}

static Mechanism_Feedback_t Test_Feedback(uint32_t now_ms)
{
    Mechanism_Feedback_t feedback = {.shaft_deg = {120.0f, -40.0f, 60.0f}};
    Test_Refresh(&feedback, now_ms);
    return feedback;
}

static void Test_Start(Mechanism_t *p_mechanism, const Mechanism_Config_t *p_config,
                        const Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    Mechanism_Init(p_mechanism, p_config, now_ms);
    assert(p_mechanism->state == MECH_STATE_UNCALIBRATED);
    assert(!p_mechanism->output.motors_enabled);
    assert(Mechanism_Enable(p_mechanism, p_feedback, now_ms) == MECH_NO_ZERO);
    assert(Mechanism_CaptureZero(p_mechanism, p_feedback, now_ms) == MECH_OK);
    assert(p_mechanism->state == MECH_STATE_READY);
    assert(Mechanism_Enable(p_mechanism, p_feedback, now_ms) == MECH_OK);
    assert(p_mechanism->state == MECH_STATE_HOLDING);
}

static void Test_Near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static void Test_Configuration(void)
{
    Mechanism_t mechanism;
    Mechanism_Config_t config = Test_Config();
    Mechanism_Feedback_t feedback = Test_Feedback(0U);
    Mechanism_Config_t invalid_config = config;
    invalid_config.arm_motor_ratio = 0.0f;
    assert(!Mechanism_ConfigValid(&invalid_config));
    Mechanism_Init(&mechanism, &invalid_config, 0U);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 0U) == MECH_BAD_CONFIG);
    assert(Mechanism_Enable(&mechanism, &feedback, 0U) == MECH_BAD_CONFIG);
    assert(Mechanism_ConfigValid(&config));
    config.right_motor_direction = 0;
    assert(!Mechanism_ConfigValid(&config));
    config = Test_Config();
    config.roll_motor_ratio = NAN;
    assert(!Mechanism_ConfigValid(&config));
    config = Test_Config();
    config.pitch_motor_ratio = 0.0f;
    assert(!Mechanism_ConfigValid(&config));
    config = Test_Config();
    config.arm_range.min_deg = 1.0f;
    assert(!Mechanism_ConfigValid(&config));
    config = Test_Config();
    config.move_time_s = 0.0f;
    assert(!Mechanism_ConfigValid(&config));
    config = Test_Config();
    config.go_speed_limit_deg_s = INFINITY;
    assert(!Mechanism_ConfigValid(&config));
    config = Test_Config();
    config.wrist_speed_limit_rpm = 0;
    assert(!Mechanism_ConfigValid(&config));
}

static Mechanism_Config_t Test_WristConfig(void)
{
    Mechanism_Config_t config = Test_Config();
    config.arm_enabled = false;
    config.arm_motor_ratio = 0.0f;
    config.arm_range = (Mechanism_Range_t){0.0f, 0.0f};
    config.go_speed_limit_deg_s = 0.0f;
    return config;
}

static void Test_WristFeedback(Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    /* GO 始终缺失且上报错误，两台 DJI 独立保持有效反馈。 */
    p_feedback->seen[MECH_GO] = false;
    p_feedback->shaft_deg[MECH_GO] = NAN;
    p_feedback->motor_error = true;
    for (unsigned i = MECH_LEFT; i < MECH_MOTOR_COUNT; ++i)
    {
        p_feedback->rx_ms[i] = now_ms;
        p_feedback->seen[i] = true;
    }
}

static void Test_WristOnlyMotion(void)
{
    Mechanism_Config_t config = Test_WristConfig();
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(0U);
    const Mechanism_Pose_t target = {0.0f, 20.0f, -10.0f};
    assert(Mechanism_ConfigValid(&config));
    /* 未启用的 GO 参数不参与计算，也不会污染腕部轨迹。 */
    config.arm_motor_ratio = NAN;
    config.arm_range = (Mechanism_Range_t){NAN, NAN};
    config.go_speed_limit_deg_s = NAN;
    assert(Mechanism_ConfigValid(&config));
    Test_WristFeedback(&feedback, 0U);
    Test_Start(&mechanism, &config, &feedback, 0U);
    Test_Near(mechanism.zero_shaft_deg[MECH_GO], 0.0f);
    Test_Near(mechanism.output.shaft_target_deg[MECH_GO], 0.0f);
    Mechanism_Pose_t invalid = target;
    invalid.arm_deg = 1.0f;
    assert(Mechanism_SetTarget(&mechanism, &invalid, 0U) == MECH_BAD_TARGET);
    invalid.arm_deg = NAN;
    assert(Mechanism_SetTarget(&mechanism, &invalid, 0U) == MECH_BAD_TARGET);
    assert(Mechanism_SetTarget(&mechanism, &target, 0U) == MECH_OK);
    Test_Near(mechanism.duration_s, 3.125f);
    for (uint32_t now_ms = 5U; now_ms <= 4000U; now_ms += 5U)
    {
        float previous_left = mechanism.output.shaft_target_deg[MECH_LEFT];
        float previous_right = mechanism.output.shaft_target_deg[MECH_RIGHT];
        feedback.shaft_deg[MECH_LEFT] = previous_left;
        feedback.shaft_deg[MECH_RIGHT] = previous_right;
        Test_WristFeedback(&feedback, now_ms);
        Mechanism_Tick(&mechanism, &feedback, now_ms);
        assert(mechanism.output.motors_enabled);
        Test_Near(mechanism.measured.arm_deg, 0.0f);
        Test_Near(mechanism.setpoint.arm_deg, 0.0f);
        Test_Near(mechanism.output.shaft_target_deg[MECH_GO], 0.0f);
        Test_Near(mechanism.output.go_speed_deg_s, 0.0f);
        Test_Near(mechanism.setpoint.pitch_deg / target.pitch_deg,
                  mechanism.setpoint.roll_deg / target.roll_deg);
        assert(fabsf(mechanism.output.shaft_target_deg[MECH_LEFT] - previous_left) / 0.005f <= 60.1f);
        assert(fabsf(mechanism.output.shaft_target_deg[MECH_RIGHT] - previous_right) / 0.005f <= 60.1f);
    }
    assert(mechanism.state == MECH_STATE_HOLDING);
    Test_Near(mechanism.output.shaft_target_deg[MECH_LEFT], 60.0f);
    Test_Near(mechanism.output.shaft_target_deg[MECH_RIGHT], 40.0f);
    Mechanism_Stop(&mechanism);
    assert(!mechanism.output.motors_enabled);
    assert(Mechanism_Enable(&mechanism, &feedback, 4000U) == MECH_OK);
    Test_Near(mechanism.measured.pitch_deg, target.pitch_deg);
    Test_Near(mechanism.measured.roll_deg, target.roll_deg);
}

static void Test_WristOnlyFaults(void)
{
    Mechanism_Config_t config = Test_WristConfig();
    for (unsigned motor = MECH_LEFT; motor < MECH_MOTOR_COUNT; ++motor)
    {
        Mechanism_t mechanism;
        Mechanism_Feedback_t feedback = Test_Feedback(0U);
        Test_WristFeedback(&feedback, 0U);
        feedback.seen[motor] = false;
        Mechanism_Init(&mechanism, &config, 0U);
        assert(Mechanism_CaptureZero(&mechanism, &feedback, 0U) == MECH_FEEDBACK_LOST);
        Test_WristFeedback(&feedback, 0U);
        Test_Start(&mechanism, &config, &feedback, 0U);
        assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_OK);
        Test_WristFeedback(&feedback, 101U);
        feedback.rx_ms[motor] = 0U;
        Mechanism_Tick(&mechanism, &feedback, 101U);
        assert(mechanism.state == MECH_STATE_FAULT && mechanism.fault == MECH_FEEDBACK_LOST);
        assert(!mechanism.output.motors_enabled && !mechanism.output.pump_on);
        Test_WristFeedback(&feedback, 105U);
        Mechanism_Stop(&mechanism);
        assert(Mechanism_Enable(&mechanism, &feedback, 105U) == MECH_NO_ZERO);
        assert(Mechanism_CaptureZero(&mechanism, &feedback, 105U) == MECH_OK);
        assert(Mechanism_Enable(&mechanism, &feedback, 105U) == MECH_OK);
        feedback.shaft_deg[motor] = NAN;
        Mechanism_Tick(&mechanism, &feedback, 110U);
        assert(mechanism.fault == MECH_FEEDBACK_LOST);
    }
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(0U);
    Test_WristFeedback(&feedback, 0U);
    Test_Start(&mechanism, &config, &feedback, 0U);
    feedback.shaft_deg[MECH_LEFT] += 600.0f;
    feedback.shaft_deg[MECH_RIGHT] -= 600.0f;
    Test_WristFeedback(&feedback, 5U);
    Mechanism_Tick(&mechanism, &feedback, 5U);
    assert(mechanism.state == MECH_STATE_FAULT && mechanism.fault == MECH_OUT_OF_RANGE);
}

static void Test_MappingAndEnable(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        Mechanism_Config_t config = Test_Config();
        config.right_motor_direction = direction;
        Mechanism_t mechanism;
        Mechanism_Feedback_t feedback = Test_Feedback(0U);
        Test_Start(&mechanism, &config, &feedback, 0U);
        Mechanism_Stop(&mechanism);
        /* 实际关节：arm=20°、pitch=10°、roll=5°；
         * 对应相对输出轴：GO=-40°、L=10°、R=direction*50°。
         */
        feedback.shaft_deg[MECH_GO] = 80.0f;
        feedback.shaft_deg[MECH_LEFT] = -30.0f;
        feedback.shaft_deg[MECH_RIGHT] = 60.0f + (float)direction * 50.0f;
        Test_Refresh(&feedback, 5U);
        assert(Mechanism_Enable(&mechanism, &feedback, 5U) == MECH_OK);
        Test_Near(mechanism.measured.arm_deg, 20.0f);
        Test_Near(mechanism.measured.pitch_deg, 10.0f);
        Test_Near(mechanism.measured.roll_deg, 5.0f);
        for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
        {
            Test_Near(mechanism.output.shaft_target_deg[i], feedback.shaft_deg[i]);
        }
        assert(Mechanism_CaptureZero(&mechanism, &feedback, 5U) == MECH_BAD_STATE);

        /* 纯俯仰到20°，保持roll=5°：L=40°、R=direction*80°。 */
        const Mechanism_Pose_t pitch_target = {20.0f, 20.0f, 5.0f};
        assert(Mechanism_SetTarget(&mechanism, &pitch_target, 5U) == MECH_OK);
        Test_Refresh(&feedback, 2005U);
        Mechanism_Tick(&mechanism, &feedback, 2005U);
        Test_Near(mechanism.output.shaft_target_deg[MECH_LEFT], 0.0f);
        Test_Near(mechanism.output.shaft_target_deg[MECH_RIGHT], 60.0f + (float)direction * 80.0f);

        /* 纯旋转到15°，保持pitch=20°：L=0°、R=direction*120°。 */
        const Mechanism_Pose_t roll_target = {20.0f, 20.0f, 15.0f};
        assert(Mechanism_SetTarget(&mechanism, &roll_target, 2005U) == MECH_OK);
        Test_Refresh(&feedback, 4005U);
        Mechanism_Tick(&mechanism, &feedback, 4005U);
        Test_Near(mechanism.output.shaft_target_deg[MECH_LEFT], -40.0f);
        Test_Near(mechanism.output.shaft_target_deg[MECH_RIGHT], 60.0f + (float)direction * 120.0f);
    }
}

static void Test_SynchronizedMotion(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(0U);
    const Mechanism_Pose_t target = {45.0f, 20.0f, -10.0f};
    Test_Start(&mechanism, &config, &feedback, 0U);
    assert(Mechanism_SetTarget(&mechanism, &target, 0U) == MECH_OK);
    assert(mechanism.state == MECH_STATE_MOVING);
    /* 左轴100°位移，在10 rpm=60 deg/s限速下至少需要3.125 s。 */
    Test_Near(mechanism.duration_s, 3.125f);
    assert(Mechanism_Enable(&mechanism, &feedback, 0U) == MECH_OK);
    assert(mechanism.state == MECH_STATE_MOVING);
    assert(Mechanism_SetTarget(&mechanism, &target, 0U) == MECH_BUSY);
    Mechanism_Pose_t invalid = target;
    invalid.pitch_deg = NAN;
    assert(Mechanism_SetTarget(&mechanism, &invalid, 0U) == MECH_BAD_TARGET);
    float previous[MECH_MOTOR_COUNT];
    for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
    {
        previous[i] = mechanism.output.shaft_target_deg[i];
    }
    for (uint32_t now_ms = 5U; now_ms <= 4000U; now_ms += 5U)
    {
        /* 模拟电机跟随上一周期目标，持续反馈。 */
        for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
        {
            feedback.shaft_deg[i] = previous[i];
        }
        Test_Refresh(&feedback, now_ms);
        Mechanism_Tick(&mechanism, &feedback, now_ms);
        assert(mechanism.output.motors_enabled);
        assert(fabsf(mechanism.output.go_speed_deg_s) <= config.go_speed_limit_deg_s + 0.001f);
        Test_Near(mechanism.setpoint.arm_deg / target.arm_deg,
                   mechanism.setpoint.pitch_deg / target.pitch_deg);
        Test_Near(mechanism.setpoint.arm_deg / target.arm_deg,
                   mechanism.setpoint.roll_deg / target.roll_deg);
        for (unsigned i = MECH_LEFT; i <= MECH_RIGHT; ++i)
        {
            float speed_deg_s = fabsf(mechanism.output.shaft_target_deg[i] - previous[i]) / 0.005f;
            assert(speed_deg_s <= 60.1f);
        }
        for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
        {
            previous[i] = mechanism.output.shaft_target_deg[i];
        }
    }
    assert(mechanism.state == MECH_STATE_HOLDING);
    assert(mechanism.setpoint.arm_deg == target.arm_deg);
    assert(mechanism.setpoint.pitch_deg == target.pitch_deg);
    assert(mechanism.setpoint.roll_deg == target.roll_deg);
    assert(mechanism.output.go_speed_deg_s == 0.0f);
    Test_Near(mechanism.output.shaft_target_deg[MECH_GO], 30.0f);
    Test_Near(mechanism.output.shaft_target_deg[MECH_LEFT], 60.0f);
    Test_Near(mechanism.output.shaft_target_deg[MECH_RIGHT], 40.0f);
    assert(Mechanism_SetTarget(&mechanism, &target, 4000U) == MECH_OK);
    assert(mechanism.state == MECH_STATE_HOLDING);
    invalid = target;
    invalid.arm_deg = 91.0f;
    assert(Mechanism_SetTarget(&mechanism, &invalid, 4000U) == MECH_BAD_TARGET);
}

static void Test_StopAndFaultRecovery(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(0U);
    const Mechanism_Pose_t target = {20.0f, 30.0f, 10.0f};
    Test_Start(&mechanism, &config, &feedback, 0U);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_OK);
    assert(Mechanism_SetTarget(&mechanism, &target, 0U) == MECH_OK);
    Mechanism_Stop(&mechanism);
    assert(mechanism.state == MECH_STATE_READY);
    assert(!mechanism.output.motors_enabled && !mechanism.output.pump_on);
    assert(mechanism.output.claw == MECH_CLAW_OFF);
    assert(Mechanism_Enable(&mechanism, &feedback, 0U) == MECH_OK);
    assert(mechanism.state == MECH_STATE_HOLDING);

    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 100U);
    assert(mechanism.output.motors_enabled); /* 超时边界仍有效 */
    Mechanism_Tick(&mechanism, &feedback, 101U);
    assert(mechanism.state == MECH_STATE_FAULT && mechanism.fault == MECH_FEEDBACK_LOST);
    assert(!mechanism.output.motors_enabled && !mechanism.output.pump_on);
    assert(mechanism.output.claw == MECH_CLAW_OFF && mechanism.output.go_speed_deg_s == 0.0f);
    Test_Refresh(&feedback, 105U);
    Mechanism_Tick(&mechanism, &feedback, 105U);
    assert(mechanism.state == MECH_STATE_FAULT);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 105U) == MECH_FEEDBACK_LOST);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_OPEN) == MECH_FEEDBACK_LOST);
    Mechanism_Stop(&mechanism);
    assert(mechanism.state == MECH_STATE_UNCALIBRATED && mechanism.fault == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 105U) == MECH_NO_ZERO);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 105U) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 105U) == MECH_OK);

    feedback.motor_error = true;
    Mechanism_Tick(&mechanism, &feedback, 110U);
    assert(mechanism.state == MECH_STATE_FAULT && mechanism.fault == MECH_MOTOR_ERROR);
    feedback.motor_error = false;
    Test_Start(&mechanism, &config, &feedback, 110U);
    feedback.shaft_deg[MECH_GO] -= 182.0f; /* 机械臂实测91°，超行程 */
    Test_Refresh(&feedback, 115U);
    assert(Mechanism_Enable(&mechanism, &feedback, 115U) == MECH_OUT_OF_RANGE);
    assert(mechanism.state == MECH_STATE_FAULT);
    Test_Start(&mechanism, &config, &feedback, 115U);
    feedback.shaft_deg[MECH_LEFT] = NAN;
    Mechanism_Tick(&mechanism, &feedback, 120U);
    assert(mechanism.fault == MECH_FEEDBACK_LOST);
}

static void Test_DisabledAirAndTickWrap(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    uint32_t start_ms = UINT32_MAX - 15U;
    Mechanism_Feedback_t feedback = Test_Feedback(start_ms);
    Mechanism_Init(&mechanism, &config, start_ms);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_OPEN) == MECH_OK);
    assert(Mechanism_SetAir(&mechanism, false, (Mechanism_Claw_t)3) == MECH_BAD_TARGET);
    assert(mechanism.output.pump_on && mechanism.output.claw == MECH_CLAW_OPEN);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, start_ms) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, start_ms + 101U);
    assert(mechanism.state == MECH_STATE_UNCALIBRATED);
    assert(mechanism.output.pump_on); /* 失能时反馈丢失仅作废零位，气路独立 */
    Test_Refresh(&feedback, 90U);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 90U) == MECH_OK);
    feedback.seen[MECH_RIGHT] = false;
    assert(Mechanism_Enable(&mechanism, &feedback, 90U) == MECH_FEEDBACK_LOST);

    feedback = Test_Feedback(start_ms);
    Test_Start(&mechanism, &config, &feedback, start_ms);
    const Mechanism_Pose_t target = {1.0f, 1.0f, 1.0f};
    assert(Mechanism_SetTarget(&mechanism, &target, start_ms) == MECH_OK);
    Test_Refresh(&feedback, start_ms + 25U);
    Mechanism_Tick(&mechanism, &feedback, start_ms + 25U);
    assert(mechanism.state == MECH_STATE_MOVING);
    Test_Near(mechanism.elapsed_s, 0.025f);
    Test_Refresh(&feedback, start_ms + 300U);
    Mechanism_Tick(&mechanism, &feedback, start_ms + 300U);
    assert(mechanism.state == MECH_STATE_HOLDING);
    assert(mechanism.output.motors_enabled);
    Mechanism_Pose_t invalid = {FLT_MAX, 0.0f, 0.0f};
    assert(Mechanism_SetTarget(&mechanism, &invalid, start_ms + 300U) == MECH_BAD_TARGET);
}

int main(void)
{
    Test_Configuration();
    Test_WristOnlyMotion();
    Test_WristOnlyFaults();
    Test_MappingAndEnable();
    Test_SynchronizedMotion();
    Test_StopAndFaultRecovery();
    Test_DisabledAirAndTickWrap();
    puts("mechanism tests passed");
    return 0;
}
