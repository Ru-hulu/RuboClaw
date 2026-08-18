# Differential-Drive MPC

这是一个独立、轻量的差速两轮机器人 MPC 控制器。它暂时不依赖 ROS、MCP、Agent
或第三方优化器，便于先验证控制闭环，后续再接入 path tracking 进程。

## 控制流程

每个控制周期调用一次 `DifferentialDriveMPC.solve(...)`：

1. 使用左右轮线速度和差速运动学模型预测未来位姿。
2. 最小化参考轨迹误差、轮速代价和轮速变化代价。
3. 施加轮速与轮加速度约束。
4. 返回预测序列中的第一条轮速命令。
5. 下一周期读取新位姿并重新求解。

## 输入

- `current_pose`：机器人当前二维位姿 `x`、`y`、`yaw`。
- `reference_poses`：从当前时刻开始的参考位姿序列。期望长度为
  `horizon + 1`，不足时自动使用最后一个位姿补齐。

## 输出

`MPCResult` 包含：

- `command.left_speed`：左轮线速度，单位 m/s。
- `command.right_speed`：右轮线速度，单位 m/s。
- `command.linear_speed`：等效车体线速度，单位 m/s。
- `command.angular_speed`：等效车体角速度，单位 rad/s。
- `predicted_poses`：当前 MPC 时域内的预测位姿。
- `cost`、`iterations`、`converged`：本次求解状态。

## 可调参数

`MPCConfig` 中的参数分为四组：

- 模型：`dt`、`horizon`、`wheel_base`。
- 约束：`max_wheel_speed`、`max_wheel_acceleration`。
- 代价：`position_weight`、`yaw_weight`、终端误差权重、轮速和轮速变化权重。
- 求解器：`optimizer_iterations`、`learning_rate`、
  `finite_difference_epsilon`、`convergence_tolerance`。

## 运行

在仓库根目录执行：

```bash
python3 -m differential_drive_mpc.demo
python3 -m unittest discover -s differential_drive_mpc/tests -v
```

## 当前边界

当前实现采用运动学模型，只负责跟踪给定参考轨迹；不处理障碍物、碰撞、轮胎打滑、
电机动力学或路径规划。这些能力应由仿真环境、导航模块或后续的动力学模型提供。
