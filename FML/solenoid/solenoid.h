/**
 * @file    solenoid.h
 * @brief   电磁阀板设备接口(FML 层)。
 *
 * 一个电磁阀板控制 4 个电磁阀,低 4 位对应通道 1..4。
 * 用队里 F4 板串口的 4Pin 口驱动(RX=SCL, TX=SDA),通路与串口复用,
 * 因此按设备而非板级资源处理。
 */
#ifndef SOLENOID_H
#define SOLENOID_H

#include <stdint.h>
#include "main.h" /* GPIO_TypeDef / GPIO_PIN_x */

#ifdef __cplusplus
extern "C" {
#endif

/* 电磁阀板引脚(与串口 4Pin 口复用) */
#define SOLENOID_BUS1_PORT GPIOA
#define SOLENOID_BUS1_SDA  GPIO_PIN_9
#define SOLENOID_BUS1_CLK  GPIO_PIN_10
#define SOLENOID_BUS2_PORT GPIOD
#define SOLENOID_BUS2_SDA  GPIO_PIN_5
#define SOLENOID_BUS2_CLK  GPIO_PIN_6
#define SOLENOID_BUS3_PORT GPIOD
#define SOLENOID_BUS3_SDA  GPIO_PIN_8
#define SOLENOID_BUS3_CLK  GPIO_PIN_9

/* 通道位掩码(低 4 位) */
#define SOLENOID_CHANNEL_1 ((uint8_t)0x01)
#define SOLENOID_CHANNEL_2 ((uint8_t)0x02)
#define SOLENOID_CHANNEL_3 ((uint8_t)0x04)
#define SOLENOID_CHANNEL_4 ((uint8_t)0x08)

typedef struct
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin_sda;
    uint16_t gpio_pin_clk;
    uint8_t data_prve;
} Solenoid_t;

typedef enum
{
    SOLENOID_BUS1 = 1,
    SOLENOID_BUS2,
    SOLENOID_BUS3,
} SolenoidBus_t;

void Solenoid_Init(SolenoidBus_t bus);
void Solenoid_Set(SolenoidBus_t bus, uint8_t cmd);
uint8_t Solenoid_Read(SolenoidBus_t bus);

#ifdef __cplusplus
}
#endif

#endif /* SOLENOID_H */
