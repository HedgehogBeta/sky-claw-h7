/** @file mechanism_config.h
 *  @brief 机构电机调参与气路接线；机械参数见同名 .c 文件。
 */
#ifndef MECHANISM_CONFIG_H
#define MECHANISM_CONFIG_H

#include "mechanism.h"

extern const Mechanism_Config_t g_mechanism_config;

/* 0：通过 Watch 直接测试模板电机对象；1：由机构任务接管电机和气路。 */
#define MECH_CONTROL_ENABLE 1

/* 0：只测试两台 M2006，GO 保持失能；1：同时控制 GO 机械臂。 */
#define MECH_GO_CONTROL_ENABLE 0

/* 左右 C610 使用 CAN ID 1、2；GO 使用 RS485 ID 0。 */
#define MECH_DJI_CURRENT_LIMIT_RAW 3000 /* C610 约 3 A */
#define MECH_DJI_POSITION_KP 0.15f       /* 位置 P 环，输出转子 rpm */
#define MECH_DJI_SPEED_KP 2.0f          /* 速度 PI 环，反馈转子 rpm */
#define MECH_DJI_SPEED_KI 0.02f
#define MECH_DJI_SPEED_FEEDFORWARD 1.0f /* 轨迹输出轴速度的前馈倍率，0 可关闭 */
/* 仅轨迹结束后启用；单位为 M2006 输出轴 deg，ENTER=0 可关闭。 */
#define MECH_DJI_HOLD_DEADBAND_ENTER_DEG 0.3f
#define MECH_DJI_HOLD_DEADBAND_EXIT_DEG 0.6f
#define MECH_GO_KP 0.0f                 /* 模板 GO 协议增益 */
#define MECH_GO_KD 0.0f

/* 泵引脚须在 CubeMX 中配置为推挽输出，初始电平为关闭电平。 */
#define MECH_PUMP_GPIO_PORT ((GPIO_TypeDef *)0)
#define MECH_PUMP_GPIO_PIN 0U
#define MECH_PUMP_ACTIVE_LEVEL GPIO_PIN_SET

/* 阀板低四位对应通道 1～4；确认与 UART 引脚的复用关系。 */
#define MECH_VALVE_BUS SOLENOID_BUS1
#define MECH_VALVE_OPEN_MASK 0U
#define MECH_VALVE_CLOSE_MASK 0U
#define MECH_VALVE_OFF_MASK 0U

#endif /* MECHANISM_CONFIG_H */
