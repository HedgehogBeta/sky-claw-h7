/** @file mechanism_app.c
 *  @brief 电机、气路适配和手动调试；模板驱动保持原有接口。
 */
#include "mechanism_app.h"
#include "DJmotor.h"
#include "UnitreeMotor.h"
#include "cmsis_os2.h"
#include "mechanism_config.h"
#include "solenoid.h"
#include "vofa.h"
#include <math.h>

#define MECH_DEG_PER_RAD 57.2957795131f
#define MECH_TASK_PERIOD_MS 5U
#define MECH_TICK_FLAG 1U
#define MECH_TASK_STACK_BYTES 2048U

#if !USE_DJ || !USE_UNITREE || MOTOR_DJI_COUNT < 2 || MOTOR_UNITREE_COUNT != 1
#error "天空爪需要两台 DJI 电机和一台 GO 电机"
#endif

#if MECH_CONTROL_ENABLE != 0 && MECH_CONTROL_ENABLE != 1
#error "MECH_CONTROL_ENABLE 必须为 0 或 1"
#endif

volatile Mechanism_Debug_t g_mechanism_debug;
/* 直接观测机构角度，不另存一份发送数据。 */
VOFA_ADD_FLOAT(0, g_mechanism_debug.measured.arm_deg);
VOFA_ADD_FLOAT(1, g_mechanism_debug.measured.pitch_deg);
VOFA_ADD_FLOAT(2, g_mechanism_debug.measured.roll_deg);
VOFA_ADD_FLOAT(3, g_mechanism_debug.setpoint.arm_deg);
VOFA_ADD_FLOAT(4, g_mechanism_debug.setpoint.pitch_deg);
VOFA_ADD_FLOAT(5, g_mechanism_debug.setpoint.roll_deg);
static Mechanism_t s_mechanism;
static osThreadId_t s_task_handle;
static volatile uint32_t s_rx_ms[MECH_MOTOR_COUNT];
static volatile bool s_seen[MECH_MOTOR_COUNT];
static bool s_motion_ready;
static bool s_air_ready;

static uint32_t Mechanism_IrqLock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

void Mechanism_DjiFeedbackCallback(const FDCAN_RxHeaderTypeDef *p_header)
{
    if (p_header->IdType == FDCAN_STANDARD_ID && p_header->RxFrameType == FDCAN_DATA_FRAME &&
        p_header->DataLength == FDCAN_DLC_BYTES_8 &&
        p_header->Identifier >= 0x201U && p_header->Identifier <= 0x202U)
    {
        unsigned motor = MECH_LEFT + p_header->Identifier - 0x201U;
        /* CAN 在电机初始化之前启动；只记录驱动已初始化、能接收的 ID。 */
        if (DJmotor[motor - MECH_LEFT].ID == p_header->Identifier - 0x200U)
        {
            s_rx_ms[motor] = HAL_GetTick();
            s_seen[motor] = true;
        }
    }
}

void Mechanism_GoReceive(UART_HandleTypeDef *p_uart)
{
    uint32_t mask = Mechanism_IrqLock();
    /* 原驱动只在发布有效反馈时赋 correct=1。
     * 用该字段判断本次是否发布，并恢复其原有锁存语义；不消费驱动的标零条件。
     */
    int previous_correct = Unitree_motors[0].data.correct;
    Unitree_motors[0].data.correct = 0;
    UnitreeMotor_UART_RxEvent(p_uart);
    if (Unitree_motors[0].data.correct == 1)
    {
        s_rx_ms[MECH_GO] = HAL_GetTick();
        s_seen[MECH_GO] = true;
    }
    else
    {
        Unitree_motors[0].data.correct = previous_correct;
    }
    __set_PRIMASK(mask);
}

static Mechanism_DebugDji_t Mechanism_ReadDji(const DJMotor *p_motor, unsigned motor, uint32_t now_ms)
{
    Mechanism_DebugDji_t debug = {
        .angle_deg = p_motor->valNow.angle_deg,
        .speed_rpm = p_motor->valNow.speed_rpm,
        .current_a = p_motor->valNow.current_A,
        .age_ms = now_ms - s_rx_ms[motor],
        .online = s_seen[motor] && now_ms - s_rx_ms[motor] <= MECH_FEEDBACK_TIMEOUT_MS};
    return debug;
}

static uint32_t Mechanism_ReadFeedback(Mechanism_Feedback_t *p_feedback)
{
    uint32_t mask = Mechanism_IrqLock();
    /* 时间、轴角与接收记录在同一临界区读取，避免把新反馈误判为超时。 */
    uint32_t now_ms = HAL_GetTick();
    const UnitreeMotorData_t *p_go = &Unitree_motors[0].data;
    Mechanism_DebugGo_t go = {
        .angle_deg = p_go->position * MECH_DEG_PER_RAD,
        .speed_deg_s = p_go->speed * MECH_DEG_PER_RAD,
        .torque_nm = p_go->torque,
        .temperature_c = p_go->temperature,
        .error = p_go->error,
        .age_ms = now_ms - s_rx_ms[MECH_GO],
        .online = s_seen[MECH_GO] && now_ms - s_rx_ms[MECH_GO] <= MECH_FEEDBACK_TIMEOUT_MS};
    Mechanism_DebugDji_t left = Mechanism_ReadDji(&DJmotor[0], MECH_LEFT, now_ms);
    Mechanism_DebugDji_t right = Mechanism_ReadDji(&DJmotor[1], MECH_RIGHT, now_ms);
    for (unsigned i = 0; i < MECH_MOTOR_COUNT; ++i)
    {
        p_feedback->rx_ms[i] = s_rx_ms[i];
        p_feedback->seen[i] = s_seen[i];
    }
    __set_PRIMASK(mask);

    p_feedback->shaft_deg[MECH_GO] = go.angle_deg;
    p_feedback->shaft_deg[MECH_LEFT] = left.angle_deg;
    p_feedback->shaft_deg[MECH_RIGHT] = right.angle_deg;
    p_feedback->motor_error = go.error != UNITREE_NO_ERR;
    g_mechanism_debug.go = go;
    g_mechanism_debug.left = left;
    g_mechanism_debug.right = right;
    return now_ms;
}

static bool Mechanism_MotorInit(void)
{
    bool valid = MECH_DJI_CURRENT_LIMIT_RAW > 0 && MECH_DJI_CURRENT_LIMIT_RAW <= 10000 &&
        isfinite(MECH_DJI_POSITION_KP) && MECH_DJI_POSITION_KP > 0.0f &&
        isfinite(MECH_DJI_SPEED_KP) && MECH_DJI_SPEED_KP > 0.0f &&
        isfinite(MECH_DJI_SPEED_KI) && MECH_DJI_SPEED_KI >= 0.0f &&
        isfinite(MECH_GO_KP) && MECH_GO_KP > 0.0f && MECH_GO_KP <= 25.599f &&
        isfinite(MECH_GO_KD) && MECH_GO_KD >= 0.0f && MECH_GO_KD <= 25.599f;
    uint32_t mask = Mechanism_IrqLock();
    for (unsigned i = 0; i < 2; ++i)
    {
        DJMotor *p_motor = &DJmotor[i];
        p_motor->param.Gear_ratio = 1.0f; /* 外部传动在机构映射中处理 */
        p_motor->param.CurrentLimit_raw = MECH_DJI_CURRENT_LIMIT_RAW;
        p_motor->limit.PosRPMFlag = true;
        /* 模板 PosRPMLimit 使用输出轴 rpm，驱动内部乘减速比。 */
        p_motor->limit.PosRPMLimit = g_mechanism_config.wrist_speed_limit_rpm;
        PID_Init(&p_motor->posPID, MECH_DJI_POSITION_KP, 0.0f, 0.0f, PIDPOS);
        PID_Init(&p_motor->velPID, MECH_DJI_SPEED_KP, MECH_DJI_SPEED_KI, 0.0f, PIDINC);
    }
    Unitree_motors[0].cmd.kp = MECH_GO_KP;
    Unitree_motors[0].cmd.kd = MECH_GO_KD;
    __set_PRIMASK(mask);
    return valid && Mechanism_ConfigValid(&g_mechanism_config);
}

static bool Mechanism_AirInit(void)
{
    GPIO_TypeDef *p_port = MECH_PUMP_GPIO_PORT;
    uint32_t pin = MECH_PUMP_GPIO_PIN;
    if (p_port == NULL || pin == 0U || pin > GPIO_PIN_15 || (pin & (pin - 1U)) != 0U ||
        (MECH_PUMP_ACTIVE_LEVEL != GPIO_PIN_SET && MECH_PUMP_ACTIVE_LEVEL != GPIO_PIN_RESET) ||
        MECH_VALVE_BUS < SOLENOID_BUS1 || MECH_VALVE_BUS > SOLENOID_BUS3 ||
        MECH_VALVE_OPEN_MASK == MECH_VALVE_CLOSE_MASK ||
        ((MECH_VALVE_OPEN_MASK | MECH_VALVE_CLOSE_MASK | MECH_VALVE_OFF_MASK) & ~0x0FU) != 0U)
    {
        return false;
    }
    GPIO_TypeDef *p_valve_port = MECH_VALVE_BUS == SOLENOID_BUS1 ? SOLENOID_BUS1_PORT :
        (MECH_VALVE_BUS == SOLENOID_BUS2 ? SOLENOID_BUS2_PORT : SOLENOID_BUS3_PORT);
    uint32_t valve_pins = MECH_VALVE_BUS == SOLENOID_BUS1 ? SOLENOID_BUS1_SDA | SOLENOID_BUS1_CLK :
        (MECH_VALVE_BUS == SOLENOID_BUS2 ? SOLENOID_BUS2_SDA | SOLENOID_BUS2_CLK :
                                         SOLENOID_BUS3_SDA | SOLENOID_BUS3_CLK);
    if (p_port == p_valve_port && (pin & valve_pins) != 0U)
    {
        return false;
    }
    /* GPIO 初始化由 CubeMX 负责，这里只接受已经配置好的推挽输出。 */
    uint32_t shift = 2U * (31U - __CLZ(pin));
    if (((p_port->MODER >> shift) & 3U) != 1U || (p_port->OTYPER & pin) != 0U)
    {
        return false;
    }
    HAL_GPIO_WritePin(p_port, pin,
                      MECH_PUMP_ACTIVE_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Solenoid_Init(MECH_VALVE_BUS);
    Solenoid_Set(MECH_VALVE_BUS, MECH_VALVE_OFF_MASK);
    return true;
}

static void Mechanism_WriteOutputs(void)
{
    const Mechanism_Output_t *p_output = &s_mechanism.output;
    uint32_t mask = Mechanism_IrqLock();
    for (unsigned i = 0; i < MOTOR_DJI_COUNT; ++i)
    {
        bool active = p_output->motors_enabled && i < 2U;
        if (active)
        {
            DJmotor[i].valSet.angle_deg = p_output->shaft_target_deg[MECH_LEFT + i];
        }
        else
        {
            DJmotor[i].valSet.current_raw = 0;
        }
        DJmotor[i].Begin = active;
        DJmotor[i].MODE_Set = active ? DJ_Position : DJ_Disable;
    }
    Unitree_motors[0].cmd.position = p_output->shaft_target_deg[MECH_GO] / MECH_DEG_PER_RAD;
    Unitree_motors[0].cmd.speed = p_output->go_speed_deg_s / MECH_DEG_PER_RAD;
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
        HAL_GPIO_WritePin(MECH_PUMP_GPIO_PORT, MECH_PUMP_GPIO_PIN,
            p_output->pump_on ? MECH_PUMP_ACTIVE_LEVEL :
            (MECH_PUMP_ACTIVE_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
        Solenoid_Set(MECH_VALVE_BUS, valve);
    }
}

static void Mechanism_ConsumeCommand(const Mechanism_Feedback_t *p_feedback, uint32_t now_ms)
{
    uint32_t command = g_mechanism_debug.command;
    Mechanism_Result_t result = MECH_OK;
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
        result = s_motion_ready ? Mechanism_CaptureZero(&s_mechanism, p_feedback, now_ms) : MECH_BAD_CONFIG;
        break;
    case MECH_CMD_ENABLE:
        result = s_motion_ready ? Mechanism_Enable(&s_mechanism, p_feedback, now_ms) : MECH_BAD_CONFIG;
        break;
    case MECH_CMD_SET_POSE:
    {
        Mechanism_Pose_t pose = g_mechanism_debug.pose;
        result = Mechanism_SetTarget(&s_mechanism, &pose, now_ms);
        break;
    }
    case MECH_CMD_SET_AIR:
        if (!s_air_ready)
        {
            result = MECH_BAD_CONFIG;
        }
        else if (g_mechanism_debug.pump_on > 1U || g_mechanism_debug.claw > MECH_CLAW_CLOSE)
        {
            result = MECH_BAD_TARGET;
        }
        else
        {
            result = Mechanism_SetAir(&s_mechanism, g_mechanism_debug.pump_on != 0U,
                                       (Mechanism_Claw_t)g_mechanism_debug.claw);
        }
        break;
    default:
        result = MECH_BAD_TARGET;
        break;
    }
    g_mechanism_debug.result = result;
    g_mechanism_debug.command = MECH_CMD_NONE;
}

static void Mechanism_UpdateDebug(void)
{
    g_mechanism_debug.state = s_mechanism.state;
    g_mechanism_debug.fault = s_mechanism.fault;
    g_mechanism_debug.measured = s_mechanism.measured;
    g_mechanism_debug.setpoint = s_mechanism.setpoint;
    g_mechanism_debug.target = s_mechanism.target;
    g_mechanism_debug.output = s_mechanism.output;
}

static void Mechanism_Task(void *argument)
{
    (void)argument;
    Mechanism_Init(&s_mechanism, &g_mechanism_config, HAL_GetTick());
    if (MECH_CONTROL_ENABLE)
    {
        s_motion_ready = Mechanism_MotorInit();
        s_air_ready = Mechanism_AirInit();
        Mechanism_WriteOutputs();
    }
    g_mechanism_debug.motion_ready = s_motion_ready;
    g_mechanism_debug.air_ready = s_air_ready;
    Mechanism_UpdateDebug();
    for (;;)
    {
        osThreadFlagsWait(MECH_TICK_FLAG, osFlagsWaitAny, osWaitForever);
        Mechanism_Feedback_t feedback;
        uint32_t now_ms = Mechanism_ReadFeedback(&feedback);
        if (MECH_CONTROL_ENABLE)
        {
            /* 先检查反馈，再处理命令；新目标不能掩盖已经发生的故障。 */
            Mechanism_Tick(&s_mechanism, &feedback, now_ms);
            Mechanism_ConsumeCommand(&feedback, now_ms);
            Mechanism_WriteOutputs();
        }
        else if (g_mechanism_debug.command != MECH_CMD_NONE)
        {
            /* 电机测试期间只观测反馈，所有控制命令均不下发。 */
            g_mechanism_debug.result = MECH_BAD_STATE;
            g_mechanism_debug.command = MECH_CMD_NONE;
        }
        Mechanism_UpdateDebug();
    }
}

void Mechanism_TaskCreate(void)
{
    const osThreadAttr_t attributes = {
        .name = "Mechanism",
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
        s_elapsed_ms = 0U;
        if (s_task_handle != NULL && osKernelGetState() == osKernelRunning)
        {
            osThreadFlagsSet(s_task_handle, MECH_TICK_FLAG);
        }
    }
}
