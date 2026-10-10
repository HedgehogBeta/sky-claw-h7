# 控制方案

## 坐标和差速关系

三轴角度均用度：

- `arm_deg`：机械臂在平面内相对机械零位的转角。
- `pitch_deg`：夹爪相对机械臂的俯仰角。
- `roll_deg`：夹爪绕自身轴的旋转角。

两侧采用相同齿轮传动。将右电机方向归一化后，同向分量对应俯仰，反向分量对应旋转：

```text
GO = zero_GO + arm_motor_ratio * arm
L  = zero_L  + pitch_motor_ratio * pitch + roll_motor_ratio * roll
R  = zero_R  + right_motor_direction *
               (pitch_motor_ratio * pitch - roll_motor_ratio * roll)
```

三个传动比带正负方向，`right_motor_direction` 为 +1 或 -1。传动比不包含 M2006 内部 36:1 和 GO 内部 6.33:1 减速比；驱动已经处理内部减速。按 CAD 齿数确定比例，再用纯俯仰、纯旋转的小角度测试确认方向。

反馈先减去标零时记录的输出轴角，再按同一关系反算三轴。机械零位由操作者放置后捕获，断电或反馈丢失后重新标零。

## 运动和状态

`mechanism.c` 处理三轴同步五次插值，起止速度与加速度均为零。运动至少持续 `move_time_s`，如果 GO 或任一 M2006 的合成位移会超过速度上限，自动延长时间。俯仰和旋转同时运动时，按左右电机实际位移检查速度。

GO 使用模板位置控制及速度前馈，M2006 使用模板位置 P 环、速度 PI 环与电流限幅。轨迹结束后保持最终目标；轨迹结束与机械实际到位分别判断，到位误差见 `measured` 和 `target`。

| state | 含义 | 下一步 |
| --- | --- | --- |
| 0 `UNCALIBRATED` | 失能、未标零 | 标零 |
| 1 `READY` | 失能、零位有效 | 使能 |
| 2 `HOLDING` | 使能、保持给定 | 设置目标或停止 |
| 3 `MOVING` | 使能、执行轨迹 | 结束后进入 HOLDING；新目标返回 BUSY |
| 4 `FAULT` | 电机和气路停止、零位失效 | 排除故障后停止、标零、使能 |

使能以当前实测姿态开始保持，不跳到历史目标。重复使能不改变正在执行的轨迹。正常停止保留有效零位；失能期间反馈丢失只作废零位，气路仍可独立控制。

使能时任何电机超过 100 ms 未收到有效反馈、GO 上报错误或实测姿态越界，进入 FAULT。反馈恢复不会自动恢复输出。停止命令清除故障记录，随后必须重新标零。停止会取消电机保持力矩，调试时应支撑机械臂。

## 先在 Watch 测试电机

默认 `mechanism_config.h` 中 `MECH_CONTROL_ENABLE=0`，机构任务只观测反馈，不覆盖模板电机对象。此阶段不需要填写机构传动比、行程或增益。打开 Keil 的 Periodic Window Update 可连续观测。

添加 `DJmotor`，`DJmotor[0]`、`DJmotor[1]` 对应 CAN3 上的 ID 1、2。反馈查看 `valNow.PulseRead`、`valNow.speed_rpm`、`valNow.angle_deg` 等字段，控制目标写入 `valSet`。

| `MODE_Set` | 模式 | 目标字段 |
| --- | --- | --- |
| 0 `DJ_Disable` | 失能，发送 0 电流 | 无 |
| 1 `DJ_RPM` | 速度 | `valSet.speed_rpm` |
| 2 `DJ_Position` | 位置 | `valSet.angle_deg` |
| 3 `DJ_Zero` | 寻零 | 使用驱动寻零参数 |
| 4 `DJ_Current` | 电流 | `valSet.current_raw` |

先写 `Begin=true` 和所需 `MODE_Set`，等待 `MODE_Cur` 与 `MODE_Set` 一致，再写目标。模板在切换模式时会将位置目标设为当前角度，并清零速度、电流及 PID 历史，因此提前写入的目标会被模式切换覆盖。停止写 `Begin=false`，驱动同时将模式恢复为 `DJ_Disable`。

GO 添加 `Unitree_motors`，查看 `data` 反馈，目标和增益在 `cmd` 中设置，通过 `enable` 控制使能；停止写 `enable=false`。各电机目标根据实际机械行程和负载确定，测试时支撑机械臂。

`g_mechanism_debug.go/left/right` 也可观测反馈。默认 `motion_ready/air_ready=false` 表示机构控制未启用；任何机构 `command` 都返回 `MECH_BAD_STATE` 并清零，不会停止或改变 Watch 中设置的电机控制。直接调试期间使用各电机的失能字段停止。

## 机构 Watch 命令

完成电机测试后，填写机械参数、增益和气路接线，将 `MECH_CONTROL_ENABLE` 改为 1，重新编译下载。机构任务开始接管电机和气路；运动或气路配置无效时，相应标零、使能或气路命令返回 `MECH_BAD_CONFIG`。

添加 `g_mechanism_debug`，先写参数，最后写 `command`；等待 `command=0` 后发送下一条。

| command | 动作 | 参数 |
| --- | --- | --- |
| 1 | 停止全部输出，清除故障 | 无 |
| 2 | 捕获机械零位 | 机构位于约定零位，电机失能 |
| 3 | 使能三台电机 | 无 |
| 4 | 设置三轴目标 | `pose.arm_deg/pitch_deg/roll_deg` |
| 5 | 设置气路 | `pump_on=0/1`，`claw=0失能/1张开/2夹紧` |

`result` 数值：0 成功、1 配置无效、2 状态不允许、3 未标零、4 反馈丢失、5 电机错误、6 目标无效、7 角度越界、8 轨迹执行中。

| 观测字段 | 用途 |
| --- | --- |
| `motion_ready/air_ready` | 机构控制已启用且对应配置可用；默认均为 false |
| `state/fault` | 当前状态及故障原因 |
| `measured/setpoint/target` | 三轴实测角、当前轨迹给定、最终目标 |
| `output` | 机构控制的电机轴角、GO 速度、使能及气路命令；仅启用机构控制时下发 |
| `go/left/right` | 电机输出轴反馈和在线状态；GO 另有温度、错误码 |
| `go/left/right.age_ms` | 收到首帧后，距最近有效反馈的时间 |

`measured` 仅在 READY、HOLDING、MOVING 状态有效。原始电机轴角在标零前也可读。

调试顺序：Watch 测试三台电机并核对反馈 → 填参数和增益 → 启用机构控制并重新编译下载 → 支撑并标零 → 使能 → 小角度机械臂运动 → 纯俯仰 → 纯旋转 → 组合运动 → 单独测试气路 → 带块测试。机构控制发生故障后执行 `1 → 2 → 3`。暂停 CPU 不会立即停止已发送的电机输出，单步前先失能电机。
