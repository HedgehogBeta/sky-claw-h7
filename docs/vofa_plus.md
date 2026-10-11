# VOFA 调试

通过 UART9（PD15 TX、PD14 RX）连接 USB-TTL 并共地。在 VOFA 中选择 115200 波特率、JustFloat 协议；当前 `APP_VOFA_ENABLE=1`，默认发送周期 10 ms。

`mechanism_app.c` 已注册以下十二路 float：

| 通道 | 观测变量 | 单位与含义 |
| --- | --- | --- |
| 0 | `g_mechanism_debug.measured.arm_deg` | deg，机械臂反馈角度 |
| 1 | `g_mechanism_debug.measured.pitch_deg` | deg，俯仰反馈角度 |
| 2 | `g_mechanism_debug.measured.roll_deg` | deg，旋转反馈角度 |
| 3 | `g_mechanism_debug.setpoint.arm_deg` | deg，机械臂轨迹位置给定 |
| 4 | `g_mechanism_debug.setpoint.pitch_deg` | deg，俯仰轨迹位置给定 |
| 5 | `g_mechanism_debug.setpoint.roll_deg` | deg，旋转轨迹位置给定 |
| 6 | `DJmotor[0].velPID.SetVal` | 转子 rpm，左电机速度给定 |
| 7 | `DJmotor[0].velPID.CurVal` | 转子 rpm，左电机速度反馈 |
| 8 | `g_mechanism_debug.left.current_a` | A，左电机实际电流 |
| 9 | `DJmotor[1].velPID.SetVal` | 转子 rpm，右电机速度给定 |
| 10 | `DJmotor[1].velPID.CurVal` | 转子 rpm，右电机速度反馈 |
| 11 | `g_mechanism_debug.right.current_a` | A，右电机实际电流 |

比较 measured 与 setpoint 的角度波形可观察跟踪误差；measured 需在标零后反馈有效时读取。腕部模式的机械臂反馈角度固定为 0，表示未控制 GO，不代表真实角度。

调速度环时同时查看 6/7、9/10 的给定与反馈，配合 8/11 的实际电流判断抖动和电流饱和。速度通道使用转子 rpm，等于 Watch 中减速器输出轴 `speed_rpm` 的 36 倍。当前增益见 [参数配置](parameters.md)。

`VOFA_MAX_CH=12`，通道 0～11 均已使用。替换变量时修改原有 `VOFA_ADD_FLOAT` 行，不重复注册；增加通道需先扩大 `VOFA_MAX_CH` 并使用未占用编号。变量须为整个任务期间有效的全局或 static float，注册在发送任务启动前完成。
