# H7 硬件与接口速查

引脚以当前 `Core/Src` 初始化代码为准，电路连接和板卡版本参照 [H7 原理图](reference/H7.pdf)。MCU 为 STM32H723ZETx，采用 LDO 供电；时钟见 `SystemClock_Config` 和 `PeriphCommonClock_Config`。

## 板载资源

LED1～4：PB3 / PB4 / PB5 / PB6，高电平点亮；蜂鸣器：PB0，高电平鸣响。固定映射位于 `HDL/bsp_board_config.h`。

## CAN

| 外设 | RX / TX | 当前用途 |
| --- | --- | --- |
| FDCAN1 | PD0 / PD1 | 用户通信预留 |
| FDCAN2 | PB12 / PB13 | 预留 |
| FDCAN3 | PF6 / PF7 | 两台 M2006 + C610，ID 1、2 |

经典 CAN，nominal 1 Mbps，无 BRS。`CAN_Start` 配置滤波并启动外设；电机反馈在 CAN 接收中断中解析，DJI 控制帧在 TIM2 中发送。DJI 保留四通道打包，未使用的 ID 3、4 为零电流。终端电阻按总线两端布置，接线参照驱动器手册。

## 串口

| 外设 | TX / RX | 波特率 | 当前用途 |
| --- | --- | --- | --- |
| USART1 | PA9 / PA10 | 460800 | 模板 IMU 接口，与阀板 BUS1 复用 |
| USART2 | PD5 / PD6 | 115200 | 预留，无 RX DMA |
| USART3 | PD8 / PD9 | 115200 | 预留，有 RX DMA |
| UART4 | PA0 / PA1 | 115200 | 预留，有 RX DMA |
| USART6 | PC6 / PC7 | 115200 | 蓝牙串口，机构控制接口尚未接入 |
| UART7 | PE8 / PE7 | 4000000 | GO-M8010-6 电机 RS485，配置 ID 0 |
| UART9 | PD15 / PD14 | 115200 | VOFA，JustFloat |

UART7 的收发方向控制为 PG1，高电平发送、低电平接收。GO 电机的电源和信号按实际型号及板卡端口确认。USB-TTL 调试接口与开发板共地。

## 气泵和阀板

泵 GPIO、有效电平和阀通道在 `APL/mechanism/mechanism_config.h`，默认未分配。泵 GPIO 在 CubeMX 配成推挽输出并以关闭电平初始化，机构应用检查后控制泵的功率驱动模块。

模板阀板驱动使用两个 GPIO 发送四位通道掩码：

| 阀板接口 | 数据 / 时钟 | 复用外设 |
| --- | --- | --- |
| BUS1 | PA9 / PA10 | USART1 |
| BUS2 | PD5 / PD6 | USART2 |
| BUS3 | PD8 / PD9 | USART3 |

使用某接口作阀板时，CubeMX 和应用配置应一起确认该接口的 GPIO 用途及串口复用。夹紧、张开与失能掩码按实际单/双线圈阀配置，并确认失能后的夹爪状态。

## 定时器

TIM1：HAL 毫秒时基；TIM2：1 kHz 电机调度，5 ms 机构任务触发。TIM3 / TIM4 / TIM5 默认未启动。
