# Mock Localization

该模块在当前原型中临时代替 SLAM 的定位输出，但不实现建图、传感器处理或定位算法。

它订阅 `/cmd_vel`，使用差速运动学模型更新内部位姿，并以
`geometry_msgs/msg/PoseStamped` 发布 `/robot_posture`。MPC 只依赖这个 Topic，
后续可以直接使用真实 SLAM 或定位节点替换本模块。

```bash
python3 -m robot_runtime.localization.mock_localization.kinematic_node
```

## MCP Tools

该模块向 RoboClaw MCP Server 注册三个进程生命周期工具：

- `start_mock_localization`
- `get_mock_localization_status`
- `stop_mock_localization`

Tool 只负责节点的启动、状态查询和停止；实时速度与位姿仍通过 ROS Topic 传递。
