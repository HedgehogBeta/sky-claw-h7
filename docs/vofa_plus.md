# VOFA 调试

模板 `FML/vofa` 通过 UART9（PD15 TX、PD14 RX，115200）发送 JustFloat，使用 USB-TTL 并共地。`APP_VOFA_ENABLE=1`，启动代码调用 `VOFA_TaskCreate(APP_VOFA_TASK_PERIOD_MS)`，默认周期 10 ms。

机构在 `mechanism_app.c` 注册以下六路 float，单位均为 deg：

| 通道 | 观测变量 |
| --- | --- |
| 0 | `g_mechanism_debug.measured.arm_deg` |
| 1 | `g_mechanism_debug.measured.pitch_deg` |
| 2 | `g_mechanism_debug.measured.roll_deg` |
| 3 | `g_mechanism_debug.setpoint.arm_deg` |
| 4 | `g_mechanism_debug.setpoint.pitch_deg` |
| 5 | `g_mechanism_debug.setpoint.roll_deg` |

measured 在标零后反馈有效时代表实际姿态，setpoint 代表当前轨迹指令。比较两组波形可观察跟踪误差和同步运动。

模板支持通道 0～7。增加观测时在文件作用域注册全局或 static 的 float，变量必须在整个任务期间有效。例如：

```c
VOFA_ADD_FLOAT(6, g_mechanism_debug.go.torque_nm);
```

发送任务每帧直接读取变量，无需业务层另存或周期复制。`VOFA_RegisterAt` 检查空指针、通道越界、重复注册和 4 字节地址对齐；动态注册应在发送任务启动前完成。`APP_VOFA_ENABLE=0` 时注册宏展开为空。

帧包含从通道 0 到最高注册通道的 float32，空槽补 0，最后附加 `00 00 80 7F`。`VOFA_GetChannelCount()` 返回帧通道数。发送任务通过 UART 的 gState 判断是否空闲，忙时跳过该帧，计数由 `VOFA_GetDropCount()` 提供；模块不依赖 UART 完成回调。

串口角色见 `FML/fm_uart_cfg.h`，DMA 内存使用模板的段与对齐约定。