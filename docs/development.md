# 修改与 CubeMX 再生成

## 修改入口

| 内容 | 位置 |
| --- | --- |
| 差速关系、行程、运动时间和速度上限 | `APL/mechanism/mechanism_config.c` |
| 机构控制开关、闭环增益、前馈、死区、电流上限、气路接线 | `APL/mechanism/mechanism_config.h` |
| 标零、轨迹、状态与故障 | `APL/mechanism/mechanism.c` |
| Watch 命令、反馈与硬件输出 | `APL/mechanism/mechanism_app.c` |
| 电机开关、数量、内部减速比与总线 | `FML/motor/motor_cfg.h` |
| UART 角色和板级资源 | `FML/fm_uart_cfg.h`、`HDL/bsp_board_config.h` |

机构控制与硬件适配保持两个职责明确的模块。模板驱动使用原有接口，新写 C 代码遵循 [C 语言规范](coding-standard.md)。

当前腕部参数已完成实机调试，日常操作使用 `g_mechanism_debug.command` 标零和控制机构。需要绕过机构层直接调试电机时，再将 `MECH_CONTROL_ENABLE` 改为 0 并重新编译下载。操作步骤见 [控制方案](control-plan.md)，当前配置见 [参数配置](parameters.md)。

## 重新生成

1. 打开 `SkyClawH7.ioc`，启用 Keep User Code；泵 GPIO 配成推挽输出，初始电平关闭。
2. 保留 `freertos.c` USER CODE 区的 `Mechanism_TaskCreate` 和 TIM2 启动；`tim.c` 的 TIM2_Init 2 区不提前启动定时器。
3. 保留 IRQ 中的机构周期、DJI 反馈时间记录及 GO 接收适配，同时保留模板原有分发。
4. 检查 Keil 的 `APL/user_ostasks/myostasks.c`、`APL/user_ostasks` 和 `APL/mechanism` 引用，机构的三个 `.c` 文件须加入工程。
5. 使用 ARM Compiler 6、`stm32h723zxx_flash.sct`，关闭 MicroLIB。
6. 保留 UART7 4 Mbps、RX Circular DMA、IDLE 中断和 PG1 方向脚；核对 TIM2 1 kHz、TIM1 HAL 时基、中断优先级。
7. 保留 AXI SRAM 可缓存、D2 SRAM Normal 不可缓存的 MPU 配置，以及 DMA 段和对齐。
8. 运行主机测试、工程引用检查与 Keil 编译，再验证实物。

验证命令见根目录 [README](../README.md)。测试覆盖差速方向与比例、三轴同步、组合运动限速、标零、停止/故障恢复、反馈超时和毫秒计数回绕。
