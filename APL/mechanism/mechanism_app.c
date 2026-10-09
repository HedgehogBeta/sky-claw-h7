/** @file mechanism_app.c
 *  @brief H7 硬件适配、手动命令及观测任务。
 */
#include "mechanism_app.h"
#include "DJmotor.h"
#include "UnitreeMotor.h"
#include "cmsis_os2.h"
#include "mechanism_board_config.h"
#include "mechanism_config.h"
#include "solenoid.h"
#include <math.h>

#define MECH_DEG_PER_RAD 57.2957795131f

volatile Mechanism_Debug_t g_mechanism_debug;
static Mechanism_t s_mechanism;
static osThreadId_t s_task_handle;
#define MECH_TICK_FLAG 1U
#define MECH_TASK_STACK_BYTES 2048U
static bool s_air_ready;

static uint32_t Mechanism_IrqLock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

/* 调用时持有中断锁，避免反馈帧更新到一半。 */
static Mechanism_DebugDji_t Mechanism_ReadDjiDebug(const DJMotor *p_motor)
{
    const Mechanism_DebugDji_t debug = {
        .angle_deg = p_motor->valNow.angle_deg,
        .speed_rpm = p_motor->valNow.speed_rpm,
        .current_a = p_motor->valNow.current_A,
        .temperature_c = p_motor->valNow.temperature_C,
        .last_rx_ms = p_motor->error.last_rx_ms,
        .received = p_motor->error.received};
    return debug;
}

static uint32_t Mechanism_ReadFeedback(Mechanism_Feedback_t *p_feedback)
{
    uint32_t mask = Mechanism_IrqLock();
    /* 时间与反馈在同一临界区捕获，避免接收时间晚于本次检查时间。 */
    uint32_t now_ms = HAL_GetTick();
    const UnitreeMotorData_t *p_go = &Unitree_motors[0].data;
    Mechanism_DebugGo_t go = {
        .angle_deg = p_go->position * MECH_DEG_PER_RAD,
        .speed_deg_s = p_go->speed * MECH_DEG_PER_RAD,
        .torque_nm = p_go->torque,
        .temperature_c = p_go->temperature,
        .error = p_go->error,
        .bad_msg = p_go->bad_msg,
        .last_rx_ms = p_go->last_rx_ms,
        .received = p_go->correct != 0};
    Mechanism_DebugDji_t left = Mechanism_ReadDjiDebug(&DJmotor[MECH_DJI_LEFT_INDEX]);
    Mechanism_DebugDji_t right = Mechanism_ReadDjiDebug(&DJmotor[MECH_DJI_RIGHT_INDEX]);
    __set_PRIMASK(mask);

    uint32_t timeout_ms = s_mechanism.config.feedback_timeout_ms;
    go.age_ms = now_ms - go.last_rx_ms;
    left.age_ms = now_ms - left.last_rx_ms;
    right.age_ms = now_ms - right.last_rx_ms;
    go.online = go.received && go.age_ms <= timeout_ms;
    left.online = left.received && left.age_ms <= timeout_ms;
    right.online = right.received && right.age_ms <= timeout_ms;
    g_mechanism_debug.go = go;
    g_mechanism_debug.left = left;
    g_mechanism_debug.right = right;

    p_feedback->shaft_deg[0] = go.angle_deg;
    p_feedback->shaft_deg[1] = left.angle_deg;
    p_feedback->shaft_deg[2] = right.angle_deg;
    p_feedback->rx_ms[0] = go.last_rx_ms;
    p_feedback->rx_ms[1] = left.last_rx_ms;
    p_feedback->rx_ms[2] = right.last_rx_ms;
    p_feedback->seen[0] = go.received;
    p_feedback->seen[1] = left.received;
    p_feedback->seen[2] = right.received;
    p_feedback->motor_error = go.error != UNITREE_NO_ERR;
    return now_ms;
}

static Mechanism_DebugPose_t Mechanism_DebugPose(const Mechanism_Pose_t *p_pose)
{
    const Mechanism_DebugPose_t debug = {
        .arm_deg = p_pose->axis[MECH_ARM],
        .pitch_deg = p_pose->axis[MECH_PITCH],
        .roll_deg = p_pose->axis[MECH_ROLL]};
    return debug;
}

static void Mechanism_UpdateDebug(void)
{
    g_mechanism_debug.state = s_mechanism.state;
    g_mechanism_debug.zero_valid = s_mechanism.zero_valid;
    g_mechanism_debug.moving = s_mechanism.moving;
    g_mechanism_debug.fault = s_mechanism.fault;
    g_mechanism_debug.measured = Mechanism_DebugPose(&s_mechanism.measured);
    g_mechanism_debug.setpoint = Mechanism_DebugPose(&s_mechanism.setpoint);
    g_mechanism_debug.target = Mechanism_DebugPose(&s_mechanism.target);
    g_mechanism_debug.output = s_mechanism.output;
}

static bool Mechanism_MotorConfigValid(void)
{
    return MECH_DJI_LEFT_INDEX != MECH_DJI_RIGHT_INDEX && MECH_DJI_LEFT_INDEX < MOTOR_DJI_COUNT &&
           MECH_DJI_RIGHT_INDEX < MOTOR_DJI_COUNT && MECH_DJI_CURRENT_LIMIT_RAW > 0 &&
           MECH_DJI_CURRENT_LIMIT_RAW <= 10000 && isfinite(MECH_DJI_POSITION_KP) &&
           MECH_DJI_POSITION_KP > 0 && isfinite(MECH_DJI_SPEED_KP) && MECH_DJI_SPEED_KP > 0 &&
           isfinite(MECH_DJI_SPEED_KI) && MECH_DJI_SPEED_KI >= 0 && isfinite(MECH_DJI_SPEED_KD) &&
           MECH_DJI_SPEED_KD >= 0 && isfinite(MECH_GO_KP) && MECH_GO_KP > 0 &&
           isfinite(MECH_GO_KD) && MECH_GO_KD >= 0;
}

static bool Mechanism_AirInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    GPIO_TypeDef *p_port = MECH_PUMP_GPIO_PORT;
    uint32_t pin = MECH_PUMP_GPIO_PIN;
    if (!MECH_PNEUMATICS_CONFIRMED || p_port == NULL || pin == 0 || pin > GPIO_PIN_15 ||
        (pin & (pin - 1U)) != 0 || MECH_VALVE_BUS < SOLENOID_BUS1 ||
        MECH_VALVE_BUS > SOLENOID_BUS3 || MECH_VALVE_OPEN_MASK == MECH_VALVE_CLOSE_MASK ||
        ((MECH_VALVE_OPEN_MASK | MECH_VALVE_CLOSE_MASK | MECH_VALVE_OFF_MASK) & ~0x0FU) != 0)
    {
        return false;
    }
    if (p_port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (p_port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (p_port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (p_port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (p_port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (p_port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (p_port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else
    {
        return false;
    }
    HAL_GPIO_WritePin(p_port, (uint16_t)pin,
                      MECH_PUMP_ACTIVE_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(p_port, &gpio);
    Solenoid_Init(MECH_VALVE_BUS);
    Solenoid_Set(MECH_VALVE_BUS, MECH_VALVE_OFF_MASK);
    return true;
}

static void Mechanism_WriteOutputs(const Mechanism_Output_t *p_output)
{
    uint32_t mask = Mechanism_IrqLock();
    for (uint32_t i = 0; i < MOTOR_DJI_COUNT; ++i)
    {
        bool active =
            p_output->motors_enabled && (i == MECH_DJI_LEFT_INDEX || i == MECH_DJI_RIGHT_INDEX);
        if (active)
        {
            unsigned shaft = i == MECH_DJI_LEFT_INDEX ? 1U : 2U;
            DJmotor[i].valSet.angle_deg = p_output->shaft_target_deg[shaft];
        }
        DJmotor[i].Begin = active;
        DJmotor[i].MODE_Set = active ? DJ_Position : DJ_Disable;
        if (!active)
        {
            DJmotor[i].valSet.current_raw = 0;
        }
    }
    Unitree_motors[0].cmd.position = p_output->shaft_target_deg[0] / MECH_DEG_PER_RAD;
    Unitree_motors[0].cmd.speed = p_output->shaft_speed_deg_s[0] / MECH_DEG_PER_RAD;
    Unitree_motors[0].cmd.torque = 0.0f;
    Unitree_motors[0].enable = p_output->motors_enabled;
    __set_PRIMASK(mask);
    if (s_air_ready)
    {
        uint8_t valve = MECH_VALVE_OFF_MASK;
        if (p_output->claw == MECH_CLAW_OPEN)
        {
            valve = MECH_VALVE_OPEN_MASK;
        }
        else if (p_output->claw == MECH_CLAW_CLOSE)
        {
            valve = MECH_VALVE_CLOSE_MASK;
        }
        HAL_GPIO_WritePin(
            MECH_PUMP_GPIO_PORT, MECH_PUMP_GPIO_PIN,
            p_output->pump_on
                ? MECH_PUMP_ACTIVE_LEVEL
                : (MECH_PUMP_ACTIVE_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
        Solenoid_Set(MECH_VALVE_BUS, valve);
    }
}

static void Mechanism_MotorInit(void)
{
    uint32_t mask = Mechanism_IrqLock();
    for (uint32_t i = 0; i < MOTOR_DJI_COUNT; ++i)
    {
        DJMotor *p_motor = &DJmotor[i];
        p_motor->param.Gear_ratio = 1.0f;
        p_motor->param.CurrentLimit_raw = MECH_DJI_CURRENT_LIMIT_RAW;
        /* 位置环输出单位为转子 rpm，内环反馈保持转子 rpm。 */
        p_motor->limit.PosRPMLimit = MECH_DJI_SPEED_LIMIT_RPM;
        p_motor->limit.PosRPMFlag = true;
        PID_Init(&p_motor->posPID, MECH_DJI_POSITION_KP, 0.0f, 0.0f, PIDPOS);
        PID_Init(&p_motor->velPID, MECH_DJI_SPEED_KP, MECH_DJI_SPEED_KI, MECH_DJI_SPEED_KD, PIDINC);
    }
    Unitree_motors[0].cmd.kp = MECH_GO_KP;
    Unitree_motors[0].cmd.kd = MECH_GO_KD;
    __set_PRIMASK(mask);
}

static void Mechanism_ConsumeCommand(const Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    Mechanism_Result_t result = MECH_OK;
    uint32_t command = g_mechanism_debug.command;
    if (command == MECH_CMD_NONE)
    {
        return;
    }
    switch (command)
    {
    case MECH_CMD_STOP:
        Mechanism_Stop(&s_mechanism);
        break;
    case MECH_CMD_CAPTURE_ZERO:
        result = Mechanism_CaptureZero(&s_mechanism, p_feedback, now_ms);
        break;
    case MECH_CMD_ENABLE:
        result = Mechanism_Enable(&s_mechanism, p_feedback, now_ms);
        break;
    case MECH_CMD_SET_POSE:
    {
        Mechanism_Pose_t pose = {
            {g_mechanism_debug.arm_deg, g_mechanism_debug.pitch_deg, g_mechanism_debug.roll_deg}};
        result = Mechanism_SetTarget(&s_mechanism, &pose, now_ms);
        break;
    }
    case MECH_CMD_SET_AIR:
        if (g_mechanism_debug.pump_on > 1U || g_mechanism_debug.claw > MECH_CLAW_CLOSE)
        {
            result = MECH_BAD_TARGET;
        }
        else
        {
            result = Mechanism_SetAir(&s_mechanism, g_mechanism_debug.pump_on != 0,
                                      (Mechanism_Claw_t)g_mechanism_debug.claw);
        }
        break;
    default:
        result = MECH_BAD_TARGET;
        break;
    }
    g_mechanism_debug.result = (uint32_t)result;
    g_mechanism_debug.completed++;
    g_mechanism_debug.command = MECH_CMD_NONE;
}

static void Mechanism_Task(void *argument)
{
    Mechanism_Config_t config = g_mechanism_config;
    (void)argument;
    Mechanism_MotorInit();
    s_air_ready = Mechanism_AirInit();
    config.pneumatics_confirmed = s_air_ready;
    config.motion_confirmed = config.motion_confirmed && Mechanism_MotorConfigValid();
    Mechanism_Init(&s_mechanism, &config, HAL_GetTick());
    g_mechanism_debug.motion_ready = config.motion_confirmed && Mechanism_ConfigValid(&config);
    g_mechanism_debug.air_ready = s_air_ready;
    Mechanism_UpdateDebug();
    for (;;)
    {
        osThreadFlagsWait(MECH_TICK_FLAG, osFlagsWaitAny, osWaitForever);
        Mechanism_Feedback_t feedback;
        uint32_t now_ms = Mechanism_ReadFeedback(&feedback);
        /* 先处理实际反馈故障，再消费命令，避免新目标掩盖故障。 */
        Mechanism_Tick(&s_mechanism, &feedback, now_ms);
        Mechanism_ConsumeCommand(&feedback, now_ms);
        Mechanism_Tick(&s_mechanism, &feedback, now_ms);
        Mechanism_WriteOutputs(&s_mechanism.output);
        Mechanism_UpdateDebug();
    }
}

void Mechanism_TaskCreate(void)
{
    const osThreadAttr_t attributes = {.name = "Mechanism",
                                       .stack_size = MECH_TASK_STACK_BYTES,
                                       .priority = osPriorityAboveNormal};
    s_task_handle = osThreadNew(Mechanism_Task, NULL, &attributes);
    if (s_task_handle == NULL)
    {
        Error_Handler();
    }
}

void Mechanism_TimerCallback(void)
{
    static uint32_t s_elapsed_ms;
    if (++s_elapsed_ms >= MECH_TASK_PERIOD_MS)
    {
        s_elapsed_ms = 0;
        if (s_task_handle != NULL && osKernelGetState() == osKernelRunning)
        {
            osThreadFlagsSet(s_task_handle, MECH_TICK_FLAG);
        }
    }
}
