# 工程结构与执行顺序

本工程沿用 H7 模板的 HDL / IRQ / FML / APL 分层，机构功能集中在 `APL/mechanism/`。

| 层 | 内容 | 主要入口 |
| --- | --- | --- |
| Core | CubeMX 初始化、HAL 中断入口、RTOS 对象创建 | `Core/Src/main.c`、`freertos.c` |
| HDL | 板级 LED、蜂鸣器、CAN 队列 | `HDL/bsp_board_config.h`、`HDL/src/` |
| IRQ | CAN / UART / TIM 回调分发 | `IRQ/src/` |
| FML | 电机、PID、CRC、蓝牙、VOFA、阀板 | `FML/motor/` 等 |
| APL | 机构逻辑、硬件适配、应用配置 | `APL/mechanism/`、`APL/app_cfg.h` |
| Drivers / Middlewares | HAL、CMSIS、FreeRTOS | 厂商库源码 |

## 启动与调度

```text
main
 ├─ HAL、时钟、GPIO、DMA、CAN、UART、TIM 初始化
 ├─ LED、蜂鸣器、CAN 启动
 ├─ DJI、Unitree 驱动初始化，电机保持失能
 ├─ 创建蓝牙模块任务
 ├─ osKernelInitialize
 ├─ MX_FREERTOS_Init
 │   ├─ 创建 AlarmTask
 │   ├─ Mechanism_TaskCreate
 │   └─ 启动 TIM2（USER CODE RTOS_THREADS 区）
 └─ osKernelStart
```

TIM2 每毫秒调用 DJI 与 Unitree 驱动。`Mechanism_TimerCallback` 每 5 ms 发一个 CMSIS-RTOS2 任务标志；机构任务等待标志后读取反馈、检查失效条件、处理手动命令并更新电机目标。轨迹计算在任务中完成，中断负责电机调度和反馈解析。

CAN 反馈在接收回调中直接解析；Unitree 使用 UART7 半双工 RS485、DMA 双缓冲及 IDLE 中断。应用读取反馈和更新电机目标时用短临界区避免中断同时修改。机构观测量每 5 ms 更新到 `g_mechanism_debug`，供 Keil Watch 查看。

TIM1 为 HAL 毫秒时基。TIM3 / TIM4 / TIM5 已初始化，默认未启动。IMU 驱动保留，默认未创建其任务。蓝牙通信任务已创建，机构命令当前使用 `g_mechanism_debug`。

## 机构模块

- `mechanism.c/.h`：显式运动状态机、零位与故障处理、腕部映射、三轴同步轨迹；状态定义在 `Mechanism_State_t`，周期状态分支在 `Mechanism_Tick`。
- `mechanism_app.c/.h`：H7 电机和气路适配、统一 Watch 结构体。
- `mechanism_config.c/.h`：外部传动关系、关节范围、速度和加速度。
- `mechanism_board_config.h`：电机增益、电流上限、泵和阀接线。

控制坐标和操作流程见 [控制方案](control-plan.md)，待定数据见 [参数清单](parameters.md)。当前状态为预调试工程，运动与气路参数确认后才能使能。

## 内存和 DMA

以 `MDK-ARM/stm32h723zxx_flash.sct`、`Core/Src/main.c` 的 MPU 配置及 `FML/includes.h` 为准。

| 区域 | 地址/大小 | 用途 |
| --- | --- | --- |
| Flash | `0x08000000`，512 KB | 固件代码和常量 |
| DTCM | `0x20000000`，124 KB | 普通变量、堆、栈 |
| EventRecorder 预留 | `0x2001F000`，4 KB | 当前组件未启用 |
| `.RAM_D1` | `0x24000000`，320 KB | AXI SRAM，可缓存 |
| `.RAM_D2` | `0x30000000`，32 KB | DMA 缓冲，MPU 配置为不可缓存 |
| `.RAM_D3` | `0x38000000`，16 KB | BDMA 可达区域 |

`main` 将运行时向量表复制到 ITCM 起始区域并设置 VTOR。DMA 缓冲按驱动约定放置；使用 AXI SRAM 时按 32 字节缓存行对齐并维护 Cache 一致性。普通数据默认使用 DTCM，外设 DMA 缓冲使用 `.RAM_D2`。
