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

参数用途见 [参数配置](docs/parameters.md)，操作步骤见 [控制方案](docs/control-plan.md)，其余资料见 [文档索引](docs/README.md)。

默认 `MECH_CONTROL_ENABLE=0`，上电电机失能。机构任务只读取反馈，不写电机参数、PID、使能或目标，也不初始化或下发机构气路。

先在 Keil Watch 添加 `DJmotor`，直接按模板方式测试电机：反馈查看 `DJmotor[i].valNow`；先写 `Begin=true` 和 `MODE_Set`，确认 `MODE_Cur` 已切换后再写 `valSet` 目标。停止写 `Begin=false`。GO 可添加 `Unitree_motors`，反馈查看 `data`，停止写 `enable=false`。此阶段不需要填写机构传动比和增益，具体字段见 [控制方案](docs/control-plan.md)。

`g_mechanism_debug` 可观测三台电机反馈；默认 `motion_ready/air_ready=false`，机构命令返回 `MECH_BAD_STATE`。完成电机测试后，填写实物参数，将 `MECH_CONTROL_ENABLE` 改为 1，重新编译下载。再将机构放到约定零位，依次执行 `command=2` 标零、`command=3` 使能；填写 `pose.arm_deg/pitch_deg/roll_deg` 后写 `command=4` 开始运动。机构控制期间停止全部输出写 `command=1`。命令消费后回到 0，结果见 `result`。

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
