# 控制方案

当前 `MECH_CONTROL_ENABLE=1`、`MECH_GO_CONTROL_ENABLE=1`，机构任务控制 GO 与两台 M2006；气路未配置。当前参数见 [参数配置](parameters.md)。

## 机构操作

在 Keil Watch 添加 `g_mechanism_debug`，启用 Periodic Window Update。每条命令先写参数、最后写 `command`；等待 `command=0` 并检查 `result=0` 后继续。

1. 将机械臂和夹爪放在约定机械零位，保持电机失能，确认 `go/left/right.online=true`。
2. 写 `command=2` 标零，再写 `command=3` 使能。
3. 填写 `pose.arm_deg/pitch_deg/roll_deg`，最后写 `command=4`。
4. 停止全部输出写 `command=1`。停止会取消保持力矩，操作前支撑夹爪。

| command | 动作 | 参数或条件 |
| --- | --- | --- |
| 1 | 停止全部输出、清除故障 | 无 |
| 2 | 捕获机械零位 | 约定零位、电机失能、反馈有效 |
| 3 | 使能受控电机 | 零位和运动配置有效 |
| 4 | 设置目标姿态 | `pose.arm_deg/pitch_deg/roll_deg` |
| 5 | 控制气路 | `pump_on=0/1`，`claw=0失能/1张开/2夹紧`；须 `air_ready=true` |

`result`：0 成功，1 配置无效，2 状态不允许，3 未标零，4 反馈丢失，5 电机错误，6 目标无效，7 角度越界，8 轨迹执行中。

## 气路

气路未配置，`air_ready=false`，`command=5` 返回配置错误，不影响三轴运动。泵 GPIO、有效电平和阀板总线、掩码在 [mechanism_config.h](../APL/mechanism/mechanism_config.h)，填写规则与启动行为见 [硬件接口](hardware_notes.md#气泵和阀板)。气路配置未就绪时机构仍可标零、使能和运动。

## 状态与保护

| state | 含义 |
| --- | --- |
| 0 `UNCALIBRATED` | 失能、未标零 |
| 1 `READY` | 失能、零位有效 |
| 2 `HOLDING` | 使能、保持给定；不保证实际到位 |
| 3 `MOVING` | 执行轨迹，新目标返回 BUSY |
| 4 `FAULT` | 电机和气路停止、零位失效 |

使能从当前实测姿态开始保持；重复使能不重启轨迹。正常停止保留零位；断电或失能期间反馈丢失后需重新标零。

受控电机超过 100 ms 无有效反馈或实测姿态越界时进入 FAULT；启用 GO 时还检查 GO 错误。反馈恢复不自动使能，排除故障后执行 `1 → 2 → 3`。暂停 CPU 不等于停止电机，单步前先失能。

常用观测：`motion_ready/air_ready` 查看配置就绪，`state/fault` 查看状态，`measured/setpoint/target` 比较反馈、轨迹给定和最终目标，`left/right` 查看电机输出轴反馈。`measured` 仅在 READY、HOLDING、MOVING 有效；波形见 [VOFA 调试](vofa_plus.md)。

## 差速与闭环

三轴角度使用 deg：`arm_deg` 为机械臂转角，`pitch_deg` 为夹爪相对机械臂的俯仰角，`roll_deg` 为夹爪绕自身轴的旋转角。

```text
GO = zero_GO + arm_motor_ratio * arm
L  = zero_L  + pitch_motor_ratio * pitch + roll_motor_ratio * roll
R  = zero_R  + right_motor_direction *
               (pitch_motor_ratio * pitch - roll_motor_ratio * roll)
```

标零记录电机输出轴角度；反馈减去零位后按同一关系反算姿态。外部传动比带方向，电机内部减速比由驱动处理。

三轴共用五次轨迹进度，起止速度和加速度为零；运动至少持续 `move_time_s`，合成电机位移超速时自动延长。规划仅约束速度，没有独立加速度约束。

M2006 控制链为“位置 P + 轨迹速度前馈 → 速度 PI → 电流限幅”；GO 按“位置与速度给定 + 协议增益”做阻抗控制，`MECH_GO_KP/KD` 为转子侧协议增益，见 [参数配置](parameters.md#go-电机参数)。左右轨迹导数经差速映射、除以 6 得到输出轴 rpm，写入 `output.dji_speed_rpm[0/1]`；驱动转换为转子 rpm 后与位置环速度相加并限速。轨迹结束、停止、故障或重新使能时前馈为零。

死区仅在 HOLDING 启用：误差达到进入阈值后，位置环按零误差计算；达到较大的退出阈值后恢复纠正。速度 PI 继续制动与承载，电机保持使能。Watch 可查看 `DJmotor[i].argum.pos_deadband_active`，阈值见 [参数配置](parameters.md)。

## 直接电机调试

机构任务接管时每 1 ms 覆写电机对象，不能再通过 Watch 直接控制；需要直接调试时把 `MECH_CONTROL_ENABLE` 改为 0 并重新编译下载，此模式机构任务只观测反馈，PID、电流上限和目标使用模板对象中的值。

`DJmotor[0/1]` 对应左右 M2006。先写 `Begin=true` 和 `MODE_Set`，等待 `MODE_Cur` 切换完成，再写目标；切模式会重置目标和 PID 历史。停止写 `Begin=false`，不能用机构命令代替。

| MODE_Set | 模式 | 目标字段 |
| --- | --- | --- |
| 0 | 失能 | 无 |
| 1 | 速度 | `valSet.speed_rpm`，输出轴 rpm |
| 2 | 位置 | `valSet.angle_deg`，输出轴 deg |
| 3 | 寻零 | 驱动寻零参数 |
| 4 | 电流 | `valSet.current_raw` |

模板初始化时速度前馈和位置死区关闭。GO 使用 `Unitree_motors[0]` 的 `cmd/data` 与 `enable`，角度、速度和力矩由驱动换算到减速器输出轴，单位与标定说明见 [参数配置](parameters.md#go-电机参数)。
