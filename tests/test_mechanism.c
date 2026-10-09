/** @file test_mechanism.c
 *  @brief 模拟三台电机反馈，验证轨迹和失效处理。
 */
#include "mechanism.h"
#include "mechanism_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static Mechanism_Config_t Test_Config(void)
{
    const Mechanism_Config_t config = {
        .motion_confirmed = true,
        .pneumatics_confirmed = true,
        .arm_motor_per_joint = -2.0f,
        .wrist = {{2.0f, 3.0f}, {2.0f, -3.0f}},
        .axis = {{-90, 90, 20, 40}, {-60, 60, 20, 40}, {-90, 90, 20, 40}},
        .wrist_motor_speed_deg_s = {25, 30},
        .wrist_motor_accel_deg_s2 = {40, 50},
        .feedback_timeout_ms = 100};
    return config;
}

static Mechanism_Feedback_t Test_Feedback(uint32_t now_ms)
{
    Mechanism_Feedback_t feedback = {
        .shaft_deg = {123, -45, 67}, .rx_ms = {now_ms, now_ms, now_ms}, .seen = {true, true, true}};
    return feedback;
}

static void Test_ConfigAndMapping(void)
{
    Mechanism_t mechanism;
    Mechanism_Config_t config = Test_Config();
    Mechanism_Feedback_t feedback = Test_Feedback(10);
    assert(Mechanism_ConfigValid(&config));
    Mechanism_Init(&mechanism, &g_mechanism_config, 10);
    assert(!mechanism.output.motors_enabled);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 10) == MECH_BAD_CONFIG);
    assert(Mechanism_Enable(&mechanism, &feedback, 10) == MECH_BAD_CONFIG);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_BAD_CONFIG);
    for (int pitch = -60; pitch <= 60; pitch += 10)
    {
        for (int roll = -90; roll <= 90; roll += 15)
        {
            float shaft[2], actual_pitch, actual_roll;
            assert(Mechanism_WristForward(&config, (float)pitch, (float)roll, shaft));
            assert(Mechanism_WristInverse(&config, shaft, &actual_pitch, &actual_roll));
            assert(fabsf(actual_pitch - pitch) < 1e-4f);
            assert(fabsf(actual_roll - roll) < 1e-4f);
        }
    }
    config.wrist[1][1] = 3;
    assert(!Mechanism_ConfigValid(&config));
    config = Test_Config();
    config.axis[MECH_ARM].max_speed_deg_s = NAN;
    assert(!Mechanism_ConfigValid(&config));
}

static void Test_Trajectory(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(100);
    Mechanism_Pose_t target = {{30, 20, -15}};
    float previous_speed[3] = {0};
    Mechanism_Init(&mechanism, &config, 100);
    assert(Mechanism_Enable(&mechanism, &feedback, 100) == MECH_NO_ZERO);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 100) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 100) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 100);
    for (unsigned i = 0; i < 3; ++i)
    {
        assert(fabsf(mechanism.output.shaft_target_deg[i] - feedback.shaft_deg[i]) < 1e-4f);
    }
    assert(Mechanism_SetTarget(&mechanism, &target, 100) == MECH_OK);
    assert(mechanism.state == MECH_STATE_MOVING);
    assert(Mechanism_SetTarget(&mechanism, &target, 100) == MECH_BUSY);
    assert(Mechanism_Enable(&mechanism, &feedback, 100) == MECH_OK);
    assert(mechanism.moving && mechanism.target.axis[MECH_ARM] == target.axis[MECH_ARM]);
    unsigned step;
    for (step = 1; step < 2000; ++step)
    {
        uint32_t now_ms = 100 + step * 5;
        for (unsigned i = 0; i < 3; ++i)
        {
            feedback.rx_ms[i] = now_ms;
        }
        Mechanism_Tick(&mechanism, &feedback, now_ms);
        assert(mechanism.fault == MECH_OK);
        assert(mechanism.output.motors_enabled);
        for (unsigned i = 0; i < 2; ++i)
        {
            float speed = mechanism.output.shaft_speed_deg_s[i + 1];
            assert(fabsf(speed) <= config.wrist_motor_speed_deg_s[i] + 0.001f);
            assert(fabsf(speed - previous_speed[i + 1]) / 0.005f <=
                   config.wrist_motor_accel_deg_s2[i] + 0.05f);
            previous_speed[i + 1] = speed;
        }
        for (unsigned i = 0; i < 3; ++i)
        {
            feedback.shaft_deg[i] = mechanism.output.shaft_target_deg[i];
        }
        if (!mechanism.moving)
        {
            break;
        }
    }
    assert(step < 2000);
    assert(mechanism.state == MECH_STATE_HOLDING);
    for (unsigned i = 0; i < MECH_AXIS_COUNT; ++i)
    {
        assert(fabsf(mechanism.setpoint.axis[i] - target.axis[i]) < 1e-4f);
    }
    assert(fabsf(mechanism.output.shaft_target_deg[0] - (123 - 60)) < 1e-4f);
    assert(fabsf(mechanism.output.shaft_target_deg[1] - (-45 + 40 - 45)) < 1e-4f);
    assert(fabsf(mechanism.output.shaft_target_deg[2] - (67 + 40 + 45)) < 1e-4f);
    target.axis[MECH_ROLL] = 91;
    assert(Mechanism_SetTarget(&mechanism, &target, 100 + step * 5) == MECH_BAD_TARGET);
    target.axis[MECH_ROLL] = NAN;
    assert(Mechanism_SetTarget(&mechanism, &target, 100 + step * 5) == MECH_BAD_TARGET);
}

static void Test_Faults(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(10);
    Mechanism_Init(&mechanism, &config, 10);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 10) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 10) == MECH_OK);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_OK);
    feedback.seen[2] = false;
    Mechanism_Tick(&mechanism, &feedback, 15);
    assert(mechanism.state == MECH_STATE_FAULT);
    assert(mechanism.fault == MECH_FEEDBACK_LOST);
    assert(!mechanism.output.motors_enabled && !mechanism.output.pump_on);
    assert(mechanism.output.claw == MECH_CLAW_OFF && !mechanism.zero_valid);
    feedback = Test_Feedback(15);
    Mechanism_Tick(&mechanism, &feedback, 15);
    assert(!mechanism.output.motors_enabled && mechanism.fault == MECH_FEEDBACK_LOST);
    assert(Mechanism_Enable(&mechanism, &feedback, 15) == MECH_FEEDBACK_LOST);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 15) == MECH_FEEDBACK_LOST);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_OPEN) == MECH_FEEDBACK_LOST);
    Mechanism_Stop(&mechanism);
    assert(mechanism.state == MECH_STATE_UNCALIBRATED);
    assert(mechanism.fault == MECH_OK && !mechanism.output.motors_enabled);
    assert(Mechanism_Enable(&mechanism, &feedback, 15) == MECH_NO_ZERO);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 15) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 15) == MECH_OK);
    feedback = Test_Feedback(2116);
    Mechanism_Tick(&mechanism, &feedback, 2116);
    assert(mechanism.fault == MECH_OK && mechanism.output.motors_enabled);
    Mechanism_Init(&mechanism, &config, 10);
    feedback = Test_Feedback(10);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 10) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 10) == MECH_OK);
    feedback.motor_error = true;
    Mechanism_Tick(&mechanism, &feedback, 11);
    assert(mechanism.fault == MECH_MOTOR_ERROR && !mechanism.output.motors_enabled);
    Mechanism_Init(&mechanism, &config, 10);
    feedback = Test_Feedback(10);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 10) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 10) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 111);
    assert(mechanism.fault == MECH_FEEDBACK_LOST);

    Mechanism_Init(&mechanism, &config, 10);
    feedback = Test_Feedback(10);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 10) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 10) == MECH_OK);
    feedback.shaft_deg[0] = 123 - 2 * 100;
    Mechanism_Tick(&mechanism, &feedback, 11);
    assert(mechanism.fault == MECH_OUT_OF_RANGE && !mechanism.output.motors_enabled);
    assert(!mechanism.zero_valid);
    Mechanism_Stop(&mechanism);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 111) == MECH_FEEDBACK_LOST);
}

/* 不再发送任何操作命令，持续反馈仍应保持电机及气路输出。 */
static void Test_HoldWithoutCommands(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(10);
    Mechanism_Init(&mechanism, &config, 10);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 10) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 10) == MECH_OK);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_OK);
    for (uint32_t now_ms = 15; now_ms <= 30010; now_ms += 5)
    {
        feedback = Test_Feedback(now_ms);
        Mechanism_Tick(&mechanism, &feedback, now_ms);
        assert(mechanism.fault == MECH_OK && mechanism.output.motors_enabled);
        assert(mechanism.output.pump_on && mechanism.output.claw == MECH_CLAW_CLOSE);
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        assert(mechanism.output.shaft_target_deg[i] == feedback.shaft_deg[i]);
        assert(mechanism.output.shaft_speed_deg_s[i] == 0);
    }
    Mechanism_Stop(&mechanism);
    assert(!mechanism.output.motors_enabled && !mechanism.output.pump_on);
    assert(mechanism.output.claw == MECH_CLAW_OFF && mechanism.zero_valid);
    assert(Mechanism_Enable(&mechanism, &feedback, 30010) == MECH_OK);
    Mechanism_Stop(&mechanism);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_OPEN) == MECH_OK);
    memset(&feedback, 0, sizeof(feedback));
    Mechanism_Tick(&mechanism, &feedback, 60010);
    assert(mechanism.fault == MECH_OK && !mechanism.output.motors_enabled);
    assert(mechanism.output.pump_on && mechanism.output.claw == MECH_CLAW_OPEN);
}

static void Test_AirAndClockWrap(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    uint32_t start_ms = UINT32_MAX - 50;
    Mechanism_Feedback_t feedback = Test_Feedback(start_ms);
    Mechanism_Init(&mechanism, &config, start_ms);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, start_ms) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, start_ms) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 25);
    assert(mechanism.fault == MECH_OK && mechanism.output.motors_enabled);
    Mechanism_Stop(&mechanism);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_OPEN) == MECH_OK);
    assert(!mechanism.output.motors_enabled && mechanism.output.pump_on);
    Mechanism_Tick(&mechanism, &feedback, 2026);
    assert(mechanism.fault == MECH_OK && mechanism.output.pump_on);
    assert(mechanism.output.claw == MECH_CLAW_OPEN);
    Mechanism_Stop(&mechanism);
    assert(!mechanism.output.pump_on && mechanism.output.claw == MECH_CLAW_OFF);
    Mechanism_Init(&mechanism, &config, start_ms);
    feedback = Test_Feedback(start_ms);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, start_ms) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, start_ms) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 49);
    assert(mechanism.fault == MECH_OK && mechanism.output.motors_enabled);
    Mechanism_Tick(&mechanism, &feedback, 50);
    assert(mechanism.fault == MECH_FEEDBACK_LOST && !mechanism.output.motors_enabled);
    Mechanism_Init(&mechanism, &config, 10);
    feedback = Test_Feedback(10);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 10) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 111);
    assert(!mechanism.zero_valid);
}

/* 通过实际命令和反馈覆盖各状态迁移，尤其是正常停止与故障恢复的区别。 */
static void Test_StateTransitions(void)
{
    Mechanism_Config_t config = Test_Config();
    Mechanism_t mechanism;
    Mechanism_Feedback_t feedback = Test_Feedback(100);
    Mechanism_Pose_t target = {{0, 0, 0}};
    Mechanism_Init(&mechanism, &config, 100);
    assert(mechanism.state == MECH_STATE_UNCALIBRATED);
    assert(!mechanism.zero_valid && !mechanism.moving && !mechanism.output.motors_enabled);
    assert(Mechanism_SetTarget(&mechanism, &target, 100) == MECH_BAD_STATE);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 100) == MECH_OK);
    assert(mechanism.state == MECH_STATE_READY && mechanism.zero_valid);
    assert(Mechanism_Enable(&mechanism, &feedback, 100) == MECH_OK);
    assert(mechanism.state == MECH_STATE_HOLDING && mechanism.output.motors_enabled);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 100) == MECH_BAD_STATE);

    /* 相同目标无须轨迹；气路命令不改变运动状态。 */
    assert(Mechanism_SetTarget(&mechanism, &target, 100) == MECH_OK);
    assert(mechanism.state == MECH_STATE_HOLDING && !mechanism.moving);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_OK);
    assert(mechanism.state == MECH_STATE_HOLDING);
    target.axis[MECH_ARM] = 10;
    assert(Mechanism_SetTarget(&mechanism, &target, 100) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 105);
    assert(mechanism.state == MECH_STATE_MOVING && mechanism.moving);
    assert(Mechanism_Enable(&mechanism, &feedback, 105) == MECH_OK);
    assert(mechanism.state == MECH_STATE_MOVING && mechanism.elapsed_s > 0);

    /* 中途正常停止保留标定，重新使能从实际姿态保持，不继续旧轨迹。 */
    Mechanism_Stop(&mechanism);
    assert(mechanism.state == MECH_STATE_READY && mechanism.zero_valid);
    assert(!mechanism.moving && !mechanism.output.motors_enabled && !mechanism.output.pump_on);
    assert(Mechanism_Enable(&mechanism, &feedback, 105) == MECH_OK);
    assert(mechanism.state == MECH_STATE_HOLDING && mechanism.target.axis[MECH_ARM] == 0);
    assert(Mechanism_SetTarget(&mechanism, &target, 105) == MECH_OK);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_CLOSE) == MECH_OK);
    feedback.motor_error = true;
    Mechanism_Tick(&mechanism, &feedback, 110);
    assert(mechanism.state == MECH_STATE_FAULT && mechanism.fault == MECH_MOTOR_ERROR);
    assert(!mechanism.zero_valid && !mechanism.moving && !mechanism.output.motors_enabled);
    assert(!mechanism.output.pump_on && mechanism.output.claw == MECH_CLAW_OFF);
    feedback = Test_Feedback(110);
    Mechanism_Tick(&mechanism, &feedback, 110);
    assert(mechanism.state == MECH_STATE_FAULT);
    Mechanism_Stop(&mechanism);
    assert(mechanism.state == MECH_STATE_UNCALIBRATED && mechanism.fault == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 110) == MECH_NO_ZERO);

    /* 待使能时丢反馈只作废零位，独立气路继续保持。 */
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 110) == MECH_OK);
    assert(Mechanism_SetAir(&mechanism, true, MECH_CLAW_OPEN) == MECH_OK);
    Mechanism_Tick(&mechanism, &feedback, 211);
    assert(mechanism.state == MECH_STATE_UNCALIBRATED && !mechanism.zero_valid);
    assert(mechanism.fault == MECH_OK && mechanism.output.pump_on);
    feedback = Test_Feedback(212);
    Mechanism_Tick(&mechanism, &feedback, 212);
    assert(mechanism.state == MECH_STATE_UNCALIBRATED);
    assert(Mechanism_Enable(&mechanism, &feedback, 212) == MECH_NO_ZERO);
    assert(Mechanism_CaptureZero(&mechanism, &feedback, 212) == MECH_OK);
    assert(Mechanism_Enable(&mechanism, &feedback, 212) == MECH_OK);
    assert(mechanism.state == MECH_STATE_HOLDING);
}

int main(void)
{
    Test_ConfigAndMapping();
    Test_Trajectory();
    Test_Faults();
    Test_HoldWithoutCommands();
    Test_AirAndClockWrap();
    Test_StateTransitions();
    puts("mechanism tests passed");
    return 0;
}
