# 工程结构

沿用 H7 模板的 HDL / IRQ / FML / APL 分层，机构应用集中在 `APL/mechanism/`。

| 层 | 内容 |
| --- | --- |
| Core | CubeMX 外设初始化、HAL 中断入口、RTOS 对象创建 |
| HDL | LED、蜂鸣器、CAN 队列、固定板级资源 |
| IRQ | CAN、UART、TIM、EXTI 回调分发 |
| FML | 模板电机、算法、蓝牙、IMU、VOFA、阀板模块 |
| APL | `user_ostasks` 中的模板 Alarm_Task；`mechanism` 中的机构控制、硬件适配与配置 |
| Drivers / Middlewares | HAL、CMSIS、FreeRTOS |

## 执行顺序

`main` 初始化外设、LED、蜂鸣器、CAN、电机和模板阀板接口，通过 `Bluetooth_TaskCreate` 和 `VOFA_TaskCreate` 创建通信任务；`Imu_TaskCreate` 按模板默认保持注释，使用 IMU 时启用。随后初始化 RTOS。`MX_FREERTOS_Init` 保留 Alarm_Task，并创建机构任务、启动 TIM2。电机初始失能。

TIM2 在电机对象初始化后启动，避免 DJI 驱动发送未初始化的 ID；调度频率和模板驱动调用保持不变。

TIM2 保留模板 CAN 出队与电机调度，每 5 ms 额外唤醒机构任务。默认 `MECH_CONTROL_ENABLE=0`，机构任务只读取反馈、更新 Watch，并以 `MECH_BAD_STATE` 拒绝机构命令；不初始化或写入机构电机参数、PID、使能、目标或气路。模板电机仍按原有周期调度，可在 Watch 中直接设置 `DJmotor` 和 `Unitree_motors`。

在 `mechanism_config.h` 将 `MECH_CONTROL_ENABLE` 改为 1 后重新编译下载，机构任务才初始化机构参数和气路，并按“读取反馈 → 检查故障和更新轨迹 → 消费命令 → 下发输出 → 更新 Watch”的顺序接管控制。CAN 与 UART 回调继续调用模板接收处理；机构适配记录有效反馈的时间。

`mechanism.c/.h` 只处理控制数学和状态，可在主机上编译；`mechanism_app.c/.h` 负责 H7 电机、GPIO、阀板和任务。`mechanism_config.c/.h` 是参数入口。

读取反馈、更新电机命令时使用短临界区，避免中断写入半帧数据。GO 接收适配借助模板 `correct` 字段识别新有效反馈，并保留该字段的锁存语义及驱动标零行为。

## 内存

以 `MDK-ARM/stm32h723zxx_flash.sct`、`main.c` 的 MPU 配置及 `FML/includes.h` 为准。

| 区域 | 地址 | 用途 |
| --- | --- | --- |
| Flash | 0x08000000 | 固件代码和常量 |
| DTCM | 0x20000000 | 普通变量、堆和栈 |
| AXI SRAM | 0x24000000 | 可缓存数据 |
| D2 SRAM | 0x30000000 | 不可缓存 DMA 缓冲 |
| D3 SRAM | 0x38000000 | BDMA 可访问区域 |

D2 SRAM 使用 Normal、不可缓存、禁止执行的内存属性。DMA 缓冲按模板驱动的段及对齐要求放置。TIM1 维护 HAL 毫秒时基；TIM3/4/5 默认未启动。

操作和状态见 [控制方案](control-plan.md)，接线见 [硬件接口](hardware_notes.md)。
