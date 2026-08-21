# Mock Localization

该模块在当前原型中临时代替 SLAM 的定位输出，但不实现建图、传感器处理或定位算法。

它订阅 `/cmd_vel`，使用差速运动学模型更新内部位姿，并以
`geometry_msgs/msg/PoseStamped` 发布 `/robot_posture`。MPC 只依赖这个 Topic，
后续可以直接使用真实 SLAM 或定位节点替换本模块。

默认初始位姿位于当前 80x80 调试地图中心：

- `x = 40.0`
- `y = 40.0`
- `yaw = 0.0`

节点同时提供一个轻量服务，用于直接读取节点内部的当前位姿：

- `/mock_localization/get_pose`
- 类型：`roboclaw_interfaces/srv/GetMockLocalizationPose`
- 响应字段：`success/message/frame_id/x/y/yaw`

```bash
python3 -m robot_runtime.localization.mock_localization.kinematic_node
```

## MCP Tools

该模块向 RoboClaw MCP Server 注册定位相关工具：

- `start_mock_localization`
- `get_mock_localization_status`
- `get_mock_localization`
- `stop_mock_localization`

Tool 负责节点的启动、状态查询、停止，以及通过 ROS service 读取当前模拟位姿。
实时速度仍通过 ROS Topic 传递。
