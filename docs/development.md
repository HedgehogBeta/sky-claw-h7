# 二次开发与 CubeMX 再生成

## 修改位置

| 修改内容 | 入口 |
| --- | --- |
| 齿轮映射、关节范围、轨迹速度/加速度 | `APL/mechanism/mechanism_config.c` |
| 电机增益、电流/速度上限、气路接线 | `APL/mechanism/mechanism_board_config.h` |
| 电机数量、内部减速比、总线 | `FML/motor/motor_cfg.h` |
| 手动指令和硬件下发 | `APL/mechanism/mechanism_app.c` |
| 板载 LED、蜂鸣器、RS485 方向引脚 | `HDL/bsp_board_config.h` |
| UART 角色映射 | `FML/fm_uart_cfg.h` |
| 外设引脚、DMA、中断优先级 | `SkyClawH7.ioc` |

应用沿用“一项机构任务、控制逻辑与硬件适配分开”的组织方式。新增功能按实际动作扩展现有模块，命名和接口见 [C 语言规范](coding-standard.md)。已移植驱动保持其接口，新写的 C 代码使用模块前缀、`_t` 类型名和最小头文件依赖。

## CubeMX 再生成

1. 使用 `SkyClawH7.ioc`，启用 Keep User Code。
2. 生成文件中的接入代码写在 USER CODE 区：`main.c` 的设备初始化、`freertos.c` 的机构任务与 TIM2 启动、HAL 回调分发。
3. 检查 Keil 工程名称、输出路径、源文件组和 include path，保留 `APL/mechanism` 与三个机构 `.c` 文件。
4. 链接器使用 `MDK-ARM/stm32h723zxx_flash.sct`，ARM Compiler 6，Use MicroLIB 关闭。
5. 保留 UART7 4 Mbps、RX Circular DMA、UART IDLE 中断及 PG1 方向脚配置。
6. 检查 TIM2 1 kHz、TIM1 HAL 时基，以及 CAN / UART / TIM 的中断优先级。
7. 保留 MPU 中 AXI SRAM 的可缓存区域和 D2 SRAM 的不可缓存区域；检查 DMA 缓冲段和对齐。
8. 运行 `python3 Tools/check_project.py`，再用 Keil 编译、链接和上板验证。

## 增加模块

- 源文件和头文件放在所属层并成对命名，加入 Keil 文件组与 include path。
- 新的电机驱动接入初始化、接收回调和周期调度；发送频率按实际协议确定。
- 新 UART 的 DMA 缓冲和接收恢复归使用该 UART 的模块；IRQ 层按句柄分发。
- 新任务在 `MX_FREERTOS_Init` 的 USER CODE 区调用模块的创建函数，长业务放在任务中。
- 机构调试量加入 `g_mechanism_debug`，由机构任务更新，供 Keil Watch 查看；字段见 [控制方案](control-plan.md)。

## 验证

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
python3 Tools/check_project.py
```

主机测试验证控制数学与状态处理。`check_project.py --arm-gcc arm-none-eabi-gcc` 可做 ARM C 语法检查；RVDS 汇编端口和完整固件链接由 Keil 验证。硬件行为以台架测试结果为准。
