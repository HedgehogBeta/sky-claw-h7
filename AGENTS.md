# 天空爪开发入口

本仓库是独立的 STM32H723 机构调试工程：GO 电机控制机械臂角度，两台 M2006 控制差速腕部，气路控制夹爪。

- 理解控制行为或修改机构逻辑时，先读 [控制方案](docs/control-plan.md) 与 [参数清单](docs/parameters.md)。
- 查找启动、任务或驱动入口时，读 [工程结构](docs/architecture.md)。修改外设或 CubeMX 配置时，再读 [硬件说明](docs/hardware_notes.md) 和 [二次开发指南](docs/development.md)。
- 新写的 C 代码遵循 [团队 C 规范](docs/coding-standard.md)，保持一项机构任务、控制逻辑与硬件适配两个职责明确的模块；公开接口只覆盖实际需求。
- 修改数学或状态行为后运行主机测试；修改工程接入后检查 Keil 引用并完成可用的编译验证。具体命令见 [README](README.md)。
- 默认 `MECH_CONTROL_ENABLE=0`，先通过 Watch 直接测试模板电机对象；机构任务只读反馈，不覆盖电机参数或控制，也不下发机构气路。启用机构控制后由机构任务接管，机械映射、零位、增益和气路接线以确认后的实物参数为准，有效配置才允许使能。

更多资料按需从 [文档索引](docs/README.md) 查阅。
