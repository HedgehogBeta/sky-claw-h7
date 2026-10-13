/** @file test_djmotor.c
 *  @brief Real DJI receive/control/packing regressions with a minimal CAN HAL.
 */
#include "DJmotor.h"
#include "mechanism_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1 = {1U};
FDCAN_HandleTypeDef hfdcan2 = {2U};
FDCAN_HandleTypeDef hfdcan3 = {3U};

static FDCAN_HandleTypeDef *last_tx_handle;
static FDCAN_TxHeaderTypeDef last_tx_header;
static uint8_t last_tx_data[8];
static unsigned tx_count;

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *handle, FDCAN_TxHeaderTypeDef *header,
    uint8_t *data)
{
    last_tx_handle = handle;
    last_tx_header = *header;
    memcpy(last_tx_data, data, sizeof(last_tx_data));
    ++tx_count;
    return HAL_OK;
}

static void Test_Reset(void)
{
    memset(DJmotor, 0, sizeof(DJmotor));
    DJmotor_Init();
    tx_count = 0U;
    last_tx_handle = NULL;
    memset(&last_tx_header, 0, sizeof(last_tx_header));
    memset(last_tx_data, 0, sizeof(last_tx_data));
}

static void Test_Near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00001f);
}

static void Test_ReceiveSpeed(unsigned index, int16_t rotor_rpm)
{
    FDCAN_RxHeaderTypeDef header = {
        .Identifier = 0x201U + index,
        .IdType = FDCAN_STANDARD_ID,
        .RxFrameType = FDCAN_DATA_FRAME};
    const uint16_t raw = (uint16_t)rotor_rpm;
    uint8_t data[8] = {0};
    data[2] = (uint8_t)(raw >> 8U);
    data[3] = (uint8_t)raw;
    DJmotor_Receive(header, data);
}

static void Test_LowSpeedFeedback(void)
{
    const int16_t rotor_rpm[] = {-36, -35, -1, 0, 1, 35, 36};
    Test_Reset();
    assert(MOTOR_M2006_REDUCTION_RATIO == 36U);
    for (unsigned motor = 0U; motor < 2U; ++motor)
    {
        for (unsigned i = 0U; i < sizeof(rotor_rpm) / sizeof(rotor_rpm[0]); ++i)
        {
            Test_ReceiveSpeed(motor, rotor_rpm[i]);
            Test_Near(DJmotor[motor].valNow.speed_rpm,
                      (float)rotor_rpm[i] / 36.0f);
        }
    }
}

static void Test_PositionBrakesLowSpeed(void)
{
    const int16_t rotor_rpm[] = {-1, 1};
    for (unsigned i = 0U; i < sizeof(rotor_rpm) / sizeof(rotor_rpm[0]); ++i)
    {
        Test_Reset();
        Test_ReceiveSpeed(0U, rotor_rpm[i]);
        DJmotor[0].Begin = true;
        /* Bypass a mode transition, which intentionally captures current pose. */
        DJmotor[0].MODE_Cur = DJ_Position;
        DJmotor[0].MODE_Set = DJ_Position;
        DJmotor[0].valSet.angle_deg = DJmotor[0].valNow.angle_deg;
        PID_Init(&DJmotor[0].posPID, MECH_DJI_POSITION_KP, 0.0f, 0.0f, PIDPOS);
        PID_Init(&DJmotor[0].velPID, MECH_DJI_SPEED_KP, MECH_DJI_SPEED_KI,
                 0.0f, PIDINC);

        /* Fractional PI increments must survive successive control cycles. */
        for (unsigned cycle = 0U; cycle < 200U; ++cycle)
        {
            DJmotor_Func();
            Test_Near(DJmotor[0].velPID.CurVal, (float)rotor_rpm[i]);
            Test_Near(DJmotor[0].velPID.SetVal, 0.0f);
        }
        assert(DJmotor[0].valSet.current_raw * rotor_rpm[i] < 0);
        assert(tx_count == 200U);
    }
}

static int16_t Test_TxCurrent(unsigned channel)
{
    const unsigned byte = channel * 2U;
    return (int16_t)(((uint16_t)last_tx_data[byte] << 8U) |
                     last_tx_data[byte + 1U]);
}

static void Test_PositionCurrentLimitAndFrame(void)
{
    Test_Reset();
    assert(MOTOR_DJI_COUNT == 4U);
    assert(MECH_DJI_CURRENT_LIMIT_RAW == 3000);
    for (unsigned i = 0U; i < 2U; ++i)
    {
        DJmotor[i].Begin = true;
        DJmotor[i].MODE_Cur = DJ_Position;
        DJmotor[i].MODE_Set = DJ_Position;
        DJmotor[i].param.CurrentLimit_raw = MECH_DJI_CURRENT_LIMIT_RAW;
        PID_Init(&DJmotor[i].posPID, MECH_DJI_POSITION_KP, 0.0f, 0.0f, PIDPOS);
        PID_Init(&DJmotor[i].velPID, MECH_DJI_SPEED_KP, MECH_DJI_SPEED_KI,
                 0.0f, PIDINC);
        DJmotor[i].limit.PosRPMLimit = 60;
        DJmotor[i].valSet.angle_deg = i == 0U ? 180.0f : -180.0f;
    }
    /* Disabled slots must overwrite even a stale nonzero command with zero. */
    DJmotor[2].valSet.current_raw = 1234;
    DJmotor[3].valSet.current_raw = -1234;

    for (unsigned cycle = 0U; cycle < 100U; ++cycle)
    {
        DJmotor_Func();
        assert(DJmotor[0].valSet.current_raw >= 0);
        assert(DJmotor[0].valSet.current_raw <= 3000);
        assert(DJmotor[1].valSet.current_raw <= 0);
        assert(DJmotor[1].valSet.current_raw >= -3000);
        assert(tx_count == cycle + 1U);
        assert(last_tx_handle == &hfdcan3);
        assert(last_tx_header.Identifier == 0x200U);
        assert(last_tx_header.IdType == FDCAN_STANDARD_ID);
        assert(last_tx_header.TxFrameType == FDCAN_DATA_FRAME);
        assert(last_tx_header.DataLength == FDCAN_DLC_BYTES_8);
        assert(Test_TxCurrent(0U) == DJmotor[0].valSet.current_raw);
        assert(Test_TxCurrent(1U) == DJmotor[1].valSet.current_raw);
        assert(Test_TxCurrent(2U) == 0);
        assert(Test_TxCurrent(3U) == 0);
    }
    assert(Test_TxCurrent(0U) == 3000);
    assert(Test_TxCurrent(1U) == -3000);

    /* The configured speed P gain must reverse current to brake this overspeed;
     * the accumulator must leave its previous saturation and remain bounded. */
    Test_ReceiveSpeed(0U, 4320);
    Test_ReceiveSpeed(1U, -4320);
    DJmotor_Func();
    assert(Test_TxCurrent(0U) < 3000);
    assert(Test_TxCurrent(1U) > -3000);
    assert(Test_TxCurrent(0U) >= -MECH_DJI_CURRENT_LIMIT_RAW);
    assert(Test_TxCurrent(1U) <= MECH_DJI_CURRENT_LIMIT_RAW);
    assert(Test_TxCurrent(0U) < 0);
    assert(Test_TxCurrent(1U) > 0);
}

static void Test_DisableAndModeSwitchClearCurrent(void)
{
    Test_Reset();
    DJmotor[0].Begin = true;
    DJmotor[0].MODE_Cur = DJ_Position;
    DJmotor[0].MODE_Set = DJ_Position;
    DJmotor[0].valSet.angle_deg = 180.0f;
    DJmotor[0].param.CurrentLimit_raw = MECH_DJI_CURRENT_LIMIT_RAW;
    DJmotor_Func();
    assert(DJmotor[0].valSet.current_raw > 0);
    assert(DJmotor[0].argum.current_accum_raw > 0.0f);

    DJmotor[0].Begin = false;
    DJmotor_Func();
    assert(Test_TxCurrent(0U) == 0);
    Test_Near(DJmotor[0].argum.current_accum_raw, 0.0f);
    assert(DJmotor[0].MODE_Cur == DJ_Disable);

    DJmotor[0].Begin = true;
    DJmotor[0].MODE_Set = DJ_Position;
    DJmotor_Func();
    assert(Test_TxCurrent(0U) == 0);
    Test_Near(DJmotor[0].argum.current_accum_raw, 0.0f);

    DJmotor[0].valSet.angle_deg = -180.0f;
    DJmotor_Func();
    assert(DJmotor[0].valSet.current_raw < 0);
    assert(DJmotor[0].argum.current_accum_raw < 0.0f);
    DJmotor[0].MODE_Set = DJ_RPM;
    DJmotor_Func();
    assert(Test_TxCurrent(0U) == 0);
    Test_Near(DJmotor[0].argum.current_accum_raw, 0.0f);
}

static void Test_DirectCurrentLimit(void)
{
    Test_Reset();
    for (unsigned i = 0U; i < 2U; ++i)
    {
        DJmotor[i].Begin = true;
        DJmotor[i].MODE_Cur = DJ_Current;
        DJmotor[i].MODE_Set = DJ_Current;
        DJmotor[i].param.CurrentLimit_raw = MECH_DJI_CURRENT_LIMIT_RAW;
        DJmotor[i].valSet.current_raw = i == 0U ? 10000 : -10000;
    }
    DJmotor_Func();
    assert(Test_TxCurrent(0U) == 3000);
    assert(Test_TxCurrent(1U) == -3000);
    assert(Test_TxCurrent(2U) == 0);
    assert(Test_TxCurrent(3U) == 0);
}

static DJMotor *Test_PositionSetup(void)
{
    Test_Reset();
    DJMotor *motor = &DJmotor[0];
    assert(motor->param.PositionSpeedFeedforward == 0.0f);
    assert(!motor->limit.PosDeadbandFlag);
    assert(!motor->argum.pos_deadband_active);
    motor->Begin = true;
    motor->MODE_Cur = DJ_Position;
    motor->MODE_Set = DJ_Position;
    motor->param.CurrentLimit_raw = MECH_DJI_CURRENT_LIMIT_RAW;
    PID_Init(&motor->posPID, MECH_DJI_POSITION_KP, 0.0f, 0.0f, PIDPOS);
    PID_Init(&motor->velPID, MECH_DJI_SPEED_KP, MECH_DJI_SPEED_KI, 0.0f, PIDINC);
    return motor;
}

static void Test_PositionSpeedFeedforward(void)
{
    const float directions[] = {-1.0f, 1.0f};
    for (unsigned i = 0U; i < sizeof(directions) / sizeof(directions[0]); ++i)
    {
        DJMotor *motor = Test_PositionSetup();
        motor->valSet.speed_rpm = directions[i] * 2.5f;
        DJmotor_Func();
        Test_Near(motor->velPID.SetVal, 0.0f); /* Template opt-in defaults off. */

        motor->param.PositionSpeedFeedforward = 1.0f;
        DJmotor_Func();
        Test_Near(motor->velPID.SetVal, directions[i] * 90.0f);
        Test_Near(motor->posPID.err[0], 0.0f);

        motor->valSet.angle_deg = directions[i];
        DJmotor_Func();
        float expected = motor->posPID.KP * (float)motor->valSet.PulseTotal +
                         directions[i] * 90.0f;
        assert(fabsf(motor->velPID.SetVal - expected) < 0.0001f);

        /* Each component is below the cap; their sum must be capped afterward. */
        motor->valSet.angle_deg = directions[i] * 0.5f;
        motor->valSet.speed_rpm = directions[i] * 0.5f;
        motor->limit.PosRPMLimit = 2;
        DJmotor_Func();
        assert(fabsf(motor->posPID.output) < 72.0f);
        assert(fabsf(motor->posPID.output) + 18.0f > 72.0f);
        Test_Near(motor->velPID.SetVal, directions[i] * 72.0f);
    }
}

static void Test_PositionDeadbandHysteresis(void)
{
    const float directions[] = {-1.0f, 1.0f};
    for (unsigned i = 0U; i < sizeof(directions) / sizeof(directions[0]); ++i)
    {
        DJMotor *motor = Test_PositionSetup();
        motor->limit.PosDeadbandFlag = true;
        motor->limit.PosDeadbandEnter_deg = 0.3f;
        motor->limit.PosDeadbandExit_deg = 0.6f;
        motor->argum.current_accum_raw = directions[i] * 1234.5f;
        motor->valSet.current_raw = (int16_t)motor->argum.current_accum_raw;

        /* Enter at the inclusive boundary, retaining the load-bearing current. */
        motor->valSet.angle_deg = directions[i] * 0.3f;
        DJmotor_Func();
        assert(motor->argum.pos_deadband_active);
        Test_Near(motor->posPID.err[0], 0.0f);
        Test_Near(motor->velPID.SetVal, 0.0f);
        Test_Near(motor->argum.current_accum_raw, directions[i] * 1234.5f);
        assert(Test_TxCurrent(0U) == (int16_t)(directions[i] * 1234.5f));

        motor->valSet.angle_deg = directions[i] * 0.45f;
        DJmotor_Func();
        assert(motor->argum.pos_deadband_active);
        Test_Near(motor->velPID.SetVal, 0.0f);

        /* Deadband suppresses position error while velocity feedback brakes. */
        motor->valNow.speed_rpm = directions[i] / 36.0f;
        for (unsigned cycle = 0U; cycle < 20U; ++cycle)
        {
            DJmotor_Func();
            assert(motor->argum.pos_deadband_active);
            Test_Near(motor->velPID.CurVal, directions[i]);
            Test_Near(motor->velPID.SetVal, 0.0f);
        }
        assert(motor->argum.current_accum_raw * directions[i] < 1234.5f);
        assert(motor->argum.current_accum_raw * directions[i] > 1200.0f);

        /* Exit at the inclusive boundary and restore position correction. */
        motor->valSet.angle_deg = directions[i] * 0.6f;
        DJmotor_Func();
        assert(!motor->argum.pos_deadband_active);
        assert(motor->posPID.err[0] * directions[i] > 0.0f);
        assert(motor->velPID.SetVal * directions[i] > 0.0f);
        motor->valSet.angle_deg = directions[i] * 0.45f;
        DJmotor_Func();
        assert(!motor->argum.pos_deadband_active);
        assert(motor->velPID.SetVal * directions[i] > 0.0f);
    }
}

static void Test_PositionDeadbandGatingAndReset(void)
{
    DJMotor *motor = Test_PositionSetup();
    motor->limit.PosDeadbandEnter_deg = 0.3f;
    motor->limit.PosDeadbandExit_deg = 0.6f;
    motor->valSet.angle_deg = 0.2f;
    DJmotor_Func();
    assert(!motor->argum.pos_deadband_active); /* Moving with flag off. */
    assert(motor->velPID.SetVal > 0.0f);

    motor->limit.PosDeadbandFlag = true;
    DJmotor_Func();
    assert(motor->argum.pos_deadband_active);
    motor->limit.PosDeadbandFlag = false;
    DJmotor_Func();
    assert(!motor->argum.pos_deadband_active);
    assert(motor->velPID.SetVal > 0.0f);

    motor->limit.PosDeadbandFlag = true;
    motor->limit.PosDeadbandEnter_deg = 0.0f;
    DJmotor_Func();
    assert(!motor->argum.pos_deadband_active);
    assert(motor->velPID.SetVal > 0.0f);

    motor->limit.PosDeadbandEnter_deg = 0.3f;
    DJmotor_Func();
    assert(motor->argum.pos_deadband_active);
    motor->MODE_Set = DJ_RPM;
    DJmotor_Func();
    assert(!motor->argum.pos_deadband_active);
    motor->MODE_Set = DJ_Position;
    DJmotor_Func();
    assert(motor->argum.pos_deadband_active);
    motor->Begin = false;
    DJmotor_Func();
    assert(!motor->argum.pos_deadband_active);
    assert(Test_TxCurrent(0U) == 0);
}

int main(void)
{
    Test_LowSpeedFeedback();
    Test_PositionBrakesLowSpeed();
    Test_PositionCurrentLimitAndFrame();
    Test_DisableAndModeSwitchClearCurrent();
    Test_DirectCurrentLimit();
    Test_PositionSpeedFeedforward();
    Test_PositionDeadbandHysteresis();
    Test_PositionDeadbandGatingAndReset();
    puts("DJI feedback precision, low-speed braking and 3 A CAN limits passed");
    return 0;
}
