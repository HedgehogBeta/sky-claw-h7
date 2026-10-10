# 蓝牙模块

保留模板 `FML/bluetooth` 的接口和协议。USART6（PC6 TX、PC7 RX，115200）用于蓝牙，`Bluetooth_TaskCreate()` 在启动调度器前创建收发队列与任务；接收任务自行启动 DMA 接收。当前机构手动命令使用 `g_mechanism_debug`。

| 文件 | 职责 |
| --- | --- |
| `bluetooth.c/.h` | UART DMA、流缓冲、帧校验、收发任务和队列 |
| `bluetooth_cfg.h` | 包头、包长、校验方式和包尾 |
| `bluetooth_serv.c/.h` | 业务结构体、命令接收和状态发送 |

业务包体直接使用 `#pragma pack(1)` 的结构体布局，按小端传输。`BT_Cmd_t` 包含 enable、auto_mode、calib、stop、slow、fast、reset 七个 bool 位域，以及 int16 的 speed、angle 和 float 的 vx；`BT_Status_t` 包含 online、ready 两个 bool 位域、int16 的 rpm 和 float 的 current。上位机应与头文件中的字段顺序、类型及帧配置一致。

帧层发送用 `Bluetooth_Send(data, length)`，接收用 `Bluetooth_Receive(&frame, timeout_ms)`；业务入口用 `BT_Serv_Status_Send`、`BT_Serv_Cmd_Receive`。接口参数以相应头文件为准。

UART IRQ 保留接收、发送完成和错误分发。UART 角色在 `FML/fm_uart_cfg.h` 中配置，DMA 缓冲、接收恢复和协议解析由模块自身处理。后续接入机构控制时，在 APL 中读取业务命令并调用机构接口。