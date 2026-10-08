/** @file mechanism_app.h
 *  @brief H7 机构任务和调试入口。
 */
#ifndef MECHANISM_APP_H
#define MECHANISM_APP_H
#include "mechanism.h"

typedef enum
{
    MECH_CMD_NONE,
    MECH_CMD_STOP,
    MECH_CMD_CAPTURE_ZERO,
    MECH_CMD_ENABLE,
    MECH_CMD_SET_POSE,
    MECH_CMD_SET_AIR,
    MECH_CMD_KEEPALIVE,
    MECH_CMD_CLEAR_FAULT
} Mechanism_Command_t;

/* 调试器先写参数，最后写 command；每条命令消费后 command 回到 NONE。 */
typedef struct
{
    uint32_t command;
    float arm_deg, pitch_deg, roll_deg;
    uint32_t pump_on, claw;
    uint32_t result, completed;
} Mechanism_Debug_t;

extern volatile Mechanism_Debug_t g_mechanism_debug;
/** @brief 创建机构任务，供 MX_FREERTOS_Init 调用。 */
void Mechanism_TaskCreate(void);
/** @brief TIM2 每毫秒调用，按机构周期发出任务标志。 */
void Mechanism_TimerCallback(void);

#endif
