# 天空爪 H7

一台 GO 电机控制机械臂在平面内旋转，两台 M2006 通过双圆锥齿轮差速控制夹爪俯仰与旋转，气路控制夹爪开合。基于 WHU_STM32H7_Template，保留模板分层、驱动接口、LED、蜂鸣器、蓝牙、IMU 和 VOFA 模块。

## 修改入口

| 内容 | 文件 |
| --- | --- |
| 传动比、三轴行程、运动时间和电机速度上限 | [mechanism_config.c](APL/mechanism/mechanism_config.c) |
| 机构控制开关、电机增益、电流上限、气泵与阀板接线 | [mechanism_config.h](APL/mechanism/mechanism_config.h) |
| 标零、状态、差速换算、同步轨迹 | [mechanism.c](APL/mechanism/mechanism.c) |
| 电机与气路适配、Watch 命令、机构任务 | [mechanism_app.c](APL/mechanism/mechanism_app.c) |
| 电机数量、总线和内部减速比 | [motor_cfg.h](FML/motor/motor_cfg.h) |

当前参数见 [参数配置](docs/parameters.md)，操作步骤见 [控制方案](docs/control-plan.md)，其余资料见 [文档索引](docs/README.md)。

当前 `MECH_CONTROL_ENABLE=1`、`MECH_GO_CONTROL_ENABLE=1`，机构任务接管三台电机；气路未配置，`air_ready=false`。在 Watch 添加 `g_mechanism_debug`，将机构和夹爪放到约定零位，依次写 `command=2` 标零、`command=3` 使能；填写 `pose.arm_deg/pitch_deg/roll_deg` 后写 `command=4`。每条命令等待 `command=0`，检查 `result` 后继续；`command=1` 停止全部输出。运动保留 100 ms 反馈超时和越界保护。气路未就绪不妨碍运动。

## 构建

固件入口：`MDK-ARM/SkyClawH7.uvprojx`，ARM Compiler 6；CubeMX 配置：`SkyClawH7.ioc`。

```powershell
powershell -File Tools/build.ps1 -UV4 "D:/Keil_v5/UV4/UV4.exe"
```

主机验证：

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
python Tools/check_project.py
```

可选 ARM C 语法检查：`python Tools/check_project.py --arm-gcc arm-none-eabi-gcc`。完整固件编译、链接由 Keil 完成；实际方向、闭环增益和气路动作需上板验证。
