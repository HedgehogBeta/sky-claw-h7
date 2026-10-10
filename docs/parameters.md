# 参数配置

默认 `mechanism_config.h` 中 `MECH_CONTROL_ENABLE=0`，用于在 Watch 中直接测试 `DJmotor` 和 `Unitree_motors`。机构参数不参与电机调试，机构任务不覆盖模板的电机参数、PID、使能或目标。完成测试后填写以下参数，将开关改为 1 并重新编译下载，才由机构任务接管电机和气路。

## 必须按实物填写

| 参数 | 位置 | 用途 |
| --- | --- | --- |
| `arm_motor_ratio` | `mechanism_config.c` | GO 输出轴到机械臂的外部传动比与方向 |
| `pitch_motor_ratio/roll_motor_ratio` | 同上 | 双圆锥齿轮的俯仰与旋转比例、左电机方向 |
| `right_motor_direction` | 同上 | 右电机轴向约定，+1 或 -1 |
| `arm_range/pitch_range/roll_range` | 同上 | 三轴实际行程，包含约定零位；旋转范围考虑气管和线束 |
| `MECH_DJI_POSITION_KP`、`MECH_DJI_SPEED_KP/KI` | `mechanism_config.h` | 两台相同 M2006 的位置 P、速度 PI 增益 |
| `MECH_GO_KP/KD` | 同上 | GO 的位置刚度与阻尼协议增益 |
| `MECH_PUMP_GPIO_PORT/PIN/ACTIVE_LEVEL` | 同上 | 泵驱动的输出引脚与开启电平；CubeMX 配成推挽输出 |
| `MECH_VALVE_BUS`、`OPEN/CLOSE/OFF_MASK` | 同上 | 阀板接口、张开/夹紧/失能通道；低四位对应四路 |

机械零位统一为三轴 0°，通过失能时的标零命令记录，不填写电机零位偏移。外部传动比不包含驱动已处理的电机内部减速比。

三个传动比和三轴行程默认 0，机构增益默认 0；启用机构控制且配置有效才能标零、使能。右电机方向默认 +1，需结合两侧轴向验证。泵引脚与阀通道默认未分配，启用机构控制、配置有效且泵 GPIO 已正确初始化后，机构气路才可用。

## 可按调试结果调整

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `move_time_s` | 2 s | 三轴每次运动的最短时间，控制起停平缓程度 |
| `go_speed_limit_deg_s` | 30 deg/s | GO 减速器输出轴的轨迹速度上限 |
| `wrist_speed_limit_rpm` | 60 rpm | 两台 M2006 输出轴的轨迹与位置环速度上限 |
| `MECH_DJI_CURRENT_LIMIT_RAW` | 1000，约 1 A | C610 电流限幅 |

速度约束按三台电机实际合成位移计算，超限时增加运动时间。带载调试先降低速度和电流，逐步确定满足动作要求的值。

## 已确定的工程设置

机构任务周期 5 ms，反馈超时 100 ms。M2006 使用 FDCAN3、CAN ID 1/2、内部减速比 36；GO-M8010-6 使用 UART7、ID 0、4 Mbps、内部减速比 6.33。电机总线编号遵循模板：1=FDCAN1、2=FDCAN2、3=FDCAN3。

接线完成后核对供电与 ID；这些设置在 `FML/motor/motor_cfg.h`。机构输入是三轴角度，按方块高度实测所需姿态后下发目标。
