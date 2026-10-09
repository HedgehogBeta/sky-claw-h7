# 控制方案

## 控制关系

| 对象 | 执行器 | 当前控制方式 |
| --- | --- | --- |
| 机械臂关节角 | GO 电机 | 位置与速度目标，使用电机内置闭环 |
| 腕部俯仰、旋转 | 两台 M2006 + C610 | 机构角度映射为电机角度，位置环套速度环，输出电流限幅 |
| 夹爪开合 | 气泵、已确认型号的阀板 | 泵开关与阀通道命令分别控制 |

应用层角度统一用度。电机驱动接口表示**电机减速器输出轴**：DJI 用度/rpm，GO 用 rad/rad/s。内部减速比在驱动中处理，外部齿轮比在机构配置中处理。

以机械臂角 `a`、相对机械臂的腕部俯仰角 `p`、腕部旋转角 `r` 为三个目标。电机输出轴目标为：

```text
GO = zero_GO + arm_motor_per_joint * a
L  = zero_L  + wrist[0][0] * p + wrist[0][1] * r
R  = zero_R  + wrist[1][0] * p + wrist[1][1] * r
```

四个腕部系数包含方向与外部传动比，需依据 CAD 和实测填写；矩阵必须可逆。机械零位由失能状态下的手动标定记录，断电或反馈丢失后重新标定。

## 程序结构

```text
TIM2（1 kHz） ── 电机发送调度
       └─ 每 5 ms 发任务标志
机构任务 ── 读取反馈 → 更新控制 → 消费手动命令 → 下发输出
```

`mechanism.c` 负责零位、状态和三轴同步轨迹；`mechanism_app.c` 负责电机/GPIO/阀板适配。五次轨迹让三个关节同步起停，并同时满足关节及两台腕部电机的速度、加速度限制。运动中返回 `MECH_BUSY`；`moving=false` 表示轨迹完成，实际到位通过反馈误差判断。

运动控制使用显式状态机，当前状态保存在 `Mechanism_t.state`，周期处理集中在 `Mechanism_Tick` 的 `switch (state)` 中。`zero_valid`、`moving` 和 `output.motors_enabled` 由状态切换函数统一同步，保留原有观测与输出接口，不作为独立状态修改。气路可在电机失能时独立工作，不改变运动状态。

| state | 状态 | 进入或离开条件 |
| --- | --- | --- |
| 0：`MECH_STATE_UNCALIBRATED` | 电机失能，未标零 | 上电、清除故障或失能时反馈丢失；标零成功进入 READY |
| 1：`MECH_STATE_READY` | 已标零，等待使能 | 标零或正常停止后进入；使能成功进入 HOLDING |
| 2：`MECH_STATE_HOLDING` | 电机使能，保持姿态 | 使能或轨迹结束后进入；接受有位移的目标进入 MOVING |
| 3：`MECH_STATE_MOVING` | 电机使能，执行轨迹 | 轨迹结束回到 HOLDING；运动中拒绝新目标 |
| 4：`MECH_STATE_FAULT` | 全部输出停止，零位失效 | 使能时反馈异常或实际越界；停止命令清除故障并进入 UNCALIBRATED |

使能前校验参数、零位和三台电机反馈，以当前姿态开始保持；重复使能不会重启轨迹。电机保持和气路输出持续到收到停止/修改命令或发生实际反馈故障，无需心跳或续期。有效反馈超时默认 100 ms；电机使能时发生反馈丢失、驱动器错误或反馈越界，停止三台电机和气路输出并记录故障，零位失效。反馈恢复不会自动恢复输出；执行停止命令清除故障记录，再重新标零、使能。正常停止保留有效零位，可以再次使能；故障后停止不恢复零位。停止输出会取消关节保持力矩，台架调试时支撑机械臂，并确认气路失能状态。

## 接线与观测

- C610：FDCAN3，CAN ID 1、2，1 Mbps；发送保留四通道，ID 3、4 为零电流。
- GO：型号已确认 GO-M8010-6；UART7 RS485，4 Mbps，配置 ID 0，内部减速比 6.33。通信和减速比见 [宇树官方参数](https://www.unitree.com/mobile/go1/motor/)，实物 ID 与接线仍需上板检查。
- 气路：按实际泵驱动、阀板和气缸配置；确认引脚与已有 UART 的复用关系后启用。

机构观测量统一放在 `g_mechanism_debug`，通过 Keil Watch 查看。

## 手动调试入口

Keil Watch 只需添加 `g_mechanism_debug`，展开即可看到全部调试命令和机构、电机观测量。打开 `View → Periodic Window Update`，在程序持续运行时观测。先写参数，再写 `command`；消费后 `command=0`，`completed` 增加，`result` 保存结果。参数和命令作为一个事务写入，等待本条完成后再发送下一条。

| 字段 | 用途 |
| --- | --- |
| `command`、`arm_deg/pitch_deg/roll_deg`、`pump_on/claw` | 可写命令区；角度单位为度 |
| `result`、`completed` | 最近一条命令的结果、已消费命令数 |
| `motion_ready`、`air_ready` | 运动配置及增益有效、气路初始化有效 |
| `state` | 显式运动状态：0 未标零、1 待使能、2 保持、3 运动、4 故障 |
| `zero_valid`、`moving`、`fault` | 零位有效、轨迹执行中、实际故障停止原因 |
| `measured`、`setpoint`、`target` | 三轴实际角、轨迹当前给定角、轨迹最终目标角；均为命名的 `arm_deg/pitch_deg/roll_deg` |
| `output` | 实际下发的 `motors_enabled`、电机输出轴目标/速度、`pump_on/claw` |
| `go` | GO 输出轴角度（deg）、速度（deg/s）、力矩、温度、驱动错误和坏帧计数 |
| `left`、`right` | 两台 M2006 输出轴角度（deg）、速度（rpm）、电流及驱动器温度字段 |

`go/left/right` 均提供 `received`（收到过有效反馈）、`last_rx_ms`（最近反馈时间）、`age_ms`（反馈距今毫秒数）和 `online`（反馈未超时）。电机角度在标零前也可读；三轴 `measured` 仅在 `zero_valid=true` 时有效。除可写命令区外，其余字段由机构任务每 5 ms 更新，作为只读观测使用。`online` 表示通信及时，电机驱动错误另看 `go.error`。

| command | 动作 | 参数 |
| --- | --- | --- |
| 1 | 停止全部输出并清除故障记录 | 无；故障后仍需重新标零 |
| 2 | 捕获机械零位 | 支撑机构并放到约定零位 |
| 3 | 使能三台电机 | 无 |
| 4 | 设置三轴目标 | `arm_deg`、`pitch_deg`、`roll_deg` |
| 5 | 设置气路 | `pump_on=0/1`；`claw=0失能/1张开/2夹紧` |

结果与故障数值见 `Mechanism_Result_t`；常用值：0 成功、1 参数未确认、2 当前操作不允许、3 未标定、4 反馈丢失、5 驱动器错误、6 目标无效、7 实际姿态越界、8 轨迹执行中。命令只保留 0～5；原续期和清故障命令已删除。

正常操作：检查 `go/left/right.online` → 填写有效配置 → 支撑机构放到零位 → `command=2` → `command=3` → 填写三个目标角 → `command=4`。需要停止时写 `command=1`；实际故障后的恢复顺序为排除故障 → `1 → 2 → 3`。Keil 暂停 CPU 不等于停止机构输出，单步调试前先停止机构并确认输出已停。

调试顺序：检查反馈 → 填参数和增益 → 支撑并标零 → 小角度机械臂运动 → 纯俯仰 → 纯旋转 → 组合运动 → 单独测试气路 → 带物料测试。当前任务提供手动调试，物料抓取成功需按实际到位/压力反馈判断。

协议参考：[DJI C610 手册](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C610%20Brushless%20DC%20Motor%20Speed%20Controller%20User%20Guide.pdf)、[Unitree SDK](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/main/README.md)。GO 增益使用协议的转子侧增益；当前力矩前馈为零。
