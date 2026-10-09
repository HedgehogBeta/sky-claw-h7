/** @file mechanism_config.c
 *  @brief 机械参数与控制限幅。
 */
#include "mechanism_config.h"
#include "mechanism_board_config.h"

/* 先填写参数并完成台架验证，再确认运动控制。 */
const Mechanism_Config_t g_mechanism_config =
{
    .motion_confirmed = false,
    .pneumatics_confirmed = MECH_PNEUMATICS_CONFIRMED,

    /* 带符号的外部传动比，内部电机减速比由驱动处理。 */
    .arm_motor_per_joint = 0.0f,
    .wrist =
    {
        {0.0f, 0.0f}, /* 左电机：俯仰系数、旋转系数 */
        {0.0f, 0.0f}  /* 右电机：俯仰系数、旋转系数 */
    },

    /* 最小角度、最大角度、最大速度、最大加速度：deg、deg/s、deg/s²。 */
    .axis =
    {
        [MECH_ARM]   = {0.0f, 0.0f, 0.0f, 0.0f},
        [MECH_PITCH] = {0.0f, 0.0f, 0.0f, 0.0f},
        [MECH_ROLL]  = {0.0f, 0.0f, 0.0f, 0.0f}
    },
    .wrist_motor_speed_deg_s = {0.0f, 0.0f},
    .wrist_motor_accel_deg_s2 = {0.0f, 0.0f},

    .feedback_timeout_ms = 100U
};
