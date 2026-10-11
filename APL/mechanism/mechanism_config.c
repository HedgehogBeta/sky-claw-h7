/** @file mechanism_config.c
 *  @brief 机械传动、行程与运动速度，全部以减速器输出轴为基准。
 */
#include "mechanism_config.h"

const Mechanism_Config_t g_mechanism_config = {
    .arm_enabled = MECH_GO_CONTROL_ENABLE != 0,
    /* 差速关系：L = pitch_ratio * pitch + roll_ratio * roll；
     *           R = right_direction * (pitch_ratio * pitch - roll_ratio * roll)。
     * 正负号按机械零位、小角度单轴测试确认；两侧使用相同齿轮传动。
     */
    .arm_motor_ratio = 0.0f,
    .pitch_motor_ratio = -1.0f,
    .roll_motor_ratio = 2.0f,
    .right_motor_direction = -1,

    /* 关节允许范围（deg），必须包含约定的 0° 机械零位。 */
    .arm_range = {0.0f, 0.0f},
    .pitch_range = {-10.0f, 190.0f},
    .roll_range = {-360.0f, 360.0f},

    /* 先用慢速调试；组合运动超过电机速度上限时自动延长运动时间。 */
    .move_time_s = 1.0f,
    .go_speed_limit_deg_s = 30.0f,
    .wrist_speed_limit_rpm = 100
};
