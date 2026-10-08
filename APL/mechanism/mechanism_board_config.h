/** @file mechanism_board_config.h
 *  @brief 电机调参和气路接线配置。
 */
#ifndef MECHANISM_BOARD_CONFIG_H
#define MECHANISM_BOARD_CONFIG_H

/* 两台 C610 的 CAN ID 为 1、2，GO 的 RS485 ID 为 0。 */
#define MECH_DJI_LEFT_INDEX 0U
#define MECH_DJI_RIGHT_INDEX 1U

/* 协议增益和电流限幅均需台架标定；1000 raw 是初期电流上限。 */
#define MECH_DJI_CURRENT_LIMIT_RAW 1000
#define MECH_DJI_SPEED_LIMIT_RPM 60
#define MECH_DJI_POSITION_KP 0.0f
#define MECH_DJI_SPEED_KP 0.0f
#define MECH_DJI_SPEED_KI 0.0f
#define MECH_DJI_SPEED_KD 0.0f
#define MECH_GO_KP 0.0f
#define MECH_GO_KD 0.0f

/* 气泵 GPIO 和电磁阀板配置，确认接线及断电状态后启用。 */
#define MECH_PNEUMATICS_CONFIRMED 0
#define MECH_PUMP_GPIO_PORT ((GPIO_TypeDef *)0)
#define MECH_PUMP_GPIO_PIN 0U
#define MECH_PUMP_ACTIVE_LEVEL GPIO_PIN_SET
#define MECH_VALVE_BUS SOLENOID_BUS1
#define MECH_VALVE_OPEN_MASK 0U
#define MECH_VALVE_CLOSE_MASK 0U
#define MECH_VALVE_OFF_MASK 0U

#define MECH_TASK_PERIOD_MS 5U

#endif
