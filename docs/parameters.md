# 参数配置

以下记录本次腕部实机调试后的配置。最新数值以 [mechanism_config.h](../APL/mechanism/mechanism_config.h) 和 [mechanism_config.c](../APL/mechanism/mechanism_config.c) 为准；更换负载、传动或启用 GO 后需重新验证。

## 当前腕部参数

当前 `MECH_CONTROL_ENABLE=1`、`MECH_GO_CONTROL_ENABLE=0`，机构任务控制两台 M2006，GO 保持失能。

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `MECH_DJI_CURRENT_LIMIT_RAW` | 3000，约 3 A | C610 电流限幅 |
| `MECH_DJI_POSITION_KP` | 0.15 | 位置 P；误差按编码器计数计算，输出转子 rpm |
| `MECH_DJI_SPEED_KP / MECH_DJI_SPEED_KI` | 2.0 / 0.02 | 速度 PI；目标与反馈使用转子 rpm |
| `MECH_DJI_SPEED_FEEDFORWARD` | 1.0 | 轨迹速度前馈倍率，0 可关闭 |
| `MECH_DJI_HOLD_DEADBAND_ENTER_DEG` | 0.3° | 保持死区进入误差，0 可关闭 |
| `MECH_DJI_HOLD_DEADBAND_EXIT_DEG` | 0.6° | 保持死区退出误差，启用时须大于进入值 |
| `move_time_s` | 1 s | 所有动作的最短时长 |
| `wrist_speed_limit_rpm` | 100 rpm | M2006 输出轴轨迹与合成速度给定上限 |
| `pitch_motor_ratio / roll_motor_ratio` | -1 / 2 | 俯仰、旋转到左电机输出轴的比例，带方向 |
| `right_motor_direction` | -1 | 右电机轴向约定 |
| `pitch_range` | -10°～190° | 俯仰允许范围 |
| `roll_range` | -360°～360° | 旋转允许范围 |

位置环 KI/KD、速度环 KD 均为 0。传动比不包含电机内部减速比；死区角度指 M2006 减速器输出轴误差，不代表夹爪端角度精度。工作原理与操作见 [控制方案](control-plan.md)。

## 总线与周期

| 项目 | 配置 |
| --- | --- |
| M2006 | FDCAN3，CAN ID 1/2，内部减速比 36 |
| GO-M8010-6 | UART7，ID 0，4 Mbps，内部减速比 6.33；当前失能 |
| 电机闭环 / 机构任务 | 1 ms / 5 ms |
| 受控电机反馈超时 | 100 ms |

总线与电机数量见 `FML/motor/motor_cfg.h`，波形通道与串口接线见 [VOFA 调试](vofa_plus.md)。

## 后续配置

- 启用 GO：填写 GO 传动比、行程和增益，将 `MECH_GO_CONTROL_ENABLE` 改为 1，再编译下载。
- 气路：配置泵 GPIO 和阀通道；当前 `air_ready=false`，腕部运动可用，气路命令不可用。
- 参数变更：每次只改一项，比较同一动作的角度、速度和电流波形。缩短时长会增加加速度需求；持续电流饱和时先处理负载或运动要求。重力前馈尚未实现。

机构配置修改后需重新编译下载。直接电机 Watch 调试使用模板参数，须单独确认 PID 和电流上限，步骤见 [控制方案](control-plan.md#直接电机调试)。
