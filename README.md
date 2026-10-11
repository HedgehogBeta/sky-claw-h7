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

当前 `MECH_CONTROL_ENABLE=1`、`MECH_GO_CONTROL_ENABLE=0`，上电电机失能，由机构任务控制两台 M2006 腕部。GO 保持失能，不要求 GO 反馈或有效 GO 参数。`g_mechanism_debug.arm_enabled` 是只读模式标志；此模式中 `pose.arm_deg` 必须为 0，`measured.arm_deg=0` 表示机械臂未受控，并非 GO 的实际角度。

填写实物腕部参数，将夹爪放到约定零位后，依次写 `command=2` 标零、`command=3` 使能；填写 `pose.pitch_deg/roll_deg`，最后写 `command=4`，两台 M2006 按同步五次轨迹运动。停止全部输出写 `command=1`。命令消费后回到 0，结果见 `result`。运动保留 100 ms 反馈超时和越界保护；`air_ready=false` 不妨碍运动命令，气路参数未填写时不可发送 `command=5`。

需要直接 Watch 测试模板电机对象时，将 `MECH_CONTROL_ENABLE` 改为 0 并重新编译下载。此时机构任务只读取反馈，不接管电机参数、PID、使能、目标或气路，机构命令返回 `MECH_BAD_STATE`。添加 `DJmotor`，先写 `Begin=true` 和 `MODE_Set`，确认 `MODE_Cur` 已切换后再写 `valSet`；停止写 `Begin=false`。恢复整机控制时，将 `MECH_GO_CONTROL_ENABLE` 改为 1，填写有效 GO 参数并重新编译下载。具体字段和步骤见 [控制方案](docs/control-plan.md)。

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
