# 天空爪 H7 调试工程

STM32H723 + FreeRTOS：一台 Unitree 控制机械臂角度，两台 M2006 差速控制腕部俯仰与旋转，气路控制夹爪开合。

## 阅读入口

完整资料见 [文档索引](docs/README.md)。

- [相对模板的改动](docs/template-changes.md)：逐项说明应用、驱动与工程接入变化。
- [控制方案](docs/control-plan.md)：控制关系、单位、任务与调试步骤。
- [待确定参数](docs/parameters.md)：机械、电机、气路和验收参数。
- [C 语言规范](docs/coding-standard.md)：本工程采用的团队规范。

## 工程入口

- 固件：`MDK-ARM/SkyClawH7.uvprojx`，使用 Keil ARM Compiler 6。
- 外设配置：`SkyClawH7.ioc`。
- 控制逻辑：`APL/mechanism/mechanism.c/.h`。
- H7 适配及手动命令：`APL/mechanism/mechanism_app.c/.h`。
- 机械参数：`APL/mechanism/mechanism_config.c`。
- 电机增益、气路接线：`APL/mechanism/mechanism_board_config.h`。

上电读取电机反馈，电机保持失能，气路保持关闭。运动参数和增益确认前，使能命令返回 `MECH_BAD_CONFIG`；气路确认前，气路命令也返回该错误。

## 编译与验证

Windows 上打开 Keil 工程编译，或运行：

```powershell
powershell -File Tools\build.ps1 -UV4 "D:\Keil_v5\UV4\UV4.exe"
```

主机端验证控制逻辑，不依赖开发板：

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
python3 Tools/check_project.py
```

可选 ARM C 语法检查：

```bash
python3 Tools/check_project.py --arm-gcc arm-none-eabi-gcc
```

CMake 仅构建主机测试。完整固件的编译、链接和下载由 Keil 完成。测试中的传动比与限幅是模拟数据，实际值填写在应用配置中。

工程基于控制组 `2027RC/WHU_STM32H7_Template`。厂商 HAL、CMSIS、FreeRTOS 文件保留其许可证声明。
