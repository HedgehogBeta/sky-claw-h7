/**
 * @file    solenoid.c
 * @brief   电磁阀板设备实现(GPIO 位带模拟串行时序)。
 */
#include "solenoid.h"

static Solenoid_t solenoid_bus1_cfg = {0};
static Solenoid_t solenoid_bus2_cfg = {0};
static Solenoid_t solenoid_bus3_cfg = {0};

static void _Solenoid_Channel_Init(Solenoid_t *solenoid, GPIO_TypeDef *gpio_port, uint16_t gpio_pin_sda, uint16_t gpio_pin_clk)
{
    solenoid->gpio_port = gpio_port;
    solenoid->gpio_pin_sda = gpio_pin_sda;
    solenoid->gpio_pin_clk = gpio_pin_clk;
    solenoid->data_prve = 0xF0;
}

void Solenoid_Init(SolenoidBus_t bus)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    switch (bus)
    {
    case SOLENOID_BUS1:
        _Solenoid_Channel_Init(&solenoid_bus1_cfg, SOLENOID_BUS1_PORT, SOLENOID_BUS1_SDA, SOLENOID_BUS1_CLK);
        __HAL_RCC_GPIOA_CLK_ENABLE();
        HAL_GPIO_WritePin(SOLENOID_BUS1_PORT, SOLENOID_BUS1_SDA | SOLENOID_BUS1_CLK, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = SOLENOID_BUS1_SDA | SOLENOID_BUS1_CLK;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;
    case SOLENOID_BUS2:
        _Solenoid_Channel_Init(&solenoid_bus2_cfg, SOLENOID_BUS2_PORT, SOLENOID_BUS2_SDA, SOLENOID_BUS2_CLK);
        __HAL_RCC_GPIOD_CLK_ENABLE();
        HAL_GPIO_WritePin(SOLENOID_BUS2_PORT, SOLENOID_BUS2_SDA | SOLENOID_BUS2_CLK, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = SOLENOID_BUS2_SDA | SOLENOID_BUS2_CLK;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;
    case SOLENOID_BUS3:
        _Solenoid_Channel_Init(&solenoid_bus3_cfg, SOLENOID_BUS3_PORT, SOLENOID_BUS3_SDA, SOLENOID_BUS3_CLK);
        __HAL_RCC_GPIOD_CLK_ENABLE();
        HAL_GPIO_WritePin(SOLENOID_BUS3_PORT, SOLENOID_BUS3_SDA | SOLENOID_BUS3_CLK, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = SOLENOID_BUS3_SDA | SOLENOID_BUS3_CLK;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;
    default:
        break;
    }
    Solenoid_Set(bus, 0);
}

static void _Solenoid_Register_Update(Solenoid_t *solenoid, uint8_t *data)
{
    if (*data == solenoid->data_prve)
        return;
    solenoid->data_prve = *data;
    for (int i = 0; i < 4; i++)
    {
        if ((*data & 0x08) == 0x08)
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_RESET);
        }
        *data <<= 1;
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
}

/* cmd 低 4 位控制,0000 4321(4321 为对应通道) */
void Solenoid_Set(SolenoidBus_t bus, uint8_t cmd)
{
    uint8_t data = cmd & 0x0f;
    switch (bus)
    {
    case SOLENOID_BUS1:
        _Solenoid_Register_Update(&solenoid_bus1_cfg, &data);
        break;
    case SOLENOID_BUS2:
        _Solenoid_Register_Update(&solenoid_bus2_cfg, &data);
        break;
    case SOLENOID_BUS3:
        _Solenoid_Register_Update(&solenoid_bus3_cfg, &data);
        break;
    default:
        break;
    }
}

uint8_t Solenoid_Read(SolenoidBus_t bus)
{
    uint8_t data = 0;
    switch (bus)
    {
    case SOLENOID_BUS1:
        data = solenoid_bus1_cfg.data_prve;
        break;
    case SOLENOID_BUS2:
        data = solenoid_bus2_cfg.data_prve;
        break;
    case SOLENOID_BUS3:
        data = solenoid_bus3_cfg.data_prve;
        break;
    default:
        break;
    }
    return data;
}
