"""Dependency-free MPC controller for a differential-drive robot."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose in metres and radians."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class WheelCommand:
    """Wheel linear speeds and their equivalent body velocity."""

    left_speed: float
    right_speed: float
    linear_speed: float
    angular_speed: float


@dataclass(frozen=True)
class MPCResult:
    """First control command and the finite-horizon prediction behind it."""

    command: WheelCommand
    predicted_poses: tuple[Pose2D, ...]
    cost: float
    iterations: int
    converged: bool


@dataclass(frozen=True)
class MPCConfig:
    """Model, constraints, cost weights, and optimizer settings."""

    dt: float = 0.1
    horizon: int = 10
    wheel_base: float = 0.45
    max_wheel_speed: float = 1.5
    max_wheel_acceleration: float = 3.0

    position_weight: float = 8.0
    yaw_weight: float = 2.0
    terminal_position_weight: float = 20.0
    terminal_yaw_weight: float = 5.0
    wheel_speed_weight: float = 0.02
    wheel_change_weight: float = 0.15

    optimizer_iterations: int = 15
    learning_rate: float = 0.08
    finite_difference_epsilon: float = 1e-3
    convergence_tolerance: float = 1e-5

    # 在配置对象创建后统一校验数值合法性，避免求解器使用无效参数。
    def __post_init__(self) -> None:
        positive_values = {
            "dt": self.dt,
            "horizon": self.horizon,
            "wheel_base": self.wheel_base,
            "max_wheel_speed": self.max_wheel_speed,
            "max_wheel_acceleration": self.max_wheel_acceleration,
            "optimizer_iterations": self.optimizer_iterations,
            "learning_rate": self.learning_rate,
            "finite_difference_epsilon": self.finite_difference_epsilon,
            "convergence_tolerance": self.convergence_tolerance,
        }
        for name, value in positive_values.items():
            if value <= 0:
                raise ValueError(f"{name} must be greater than zero")

        weights = {
            "position_weight": self.position_weight,
            "yaw_weight": self.yaw_weight,
            "terminal_position_weight": self.terminal_position_weight,
            "terminal_yaw_weight": self.terminal_yaw_weight,
            "wheel_speed_weight": self.wheel_speed_weight,
            "wheel_change_weight": self.wheel_change_weight,
        }
        for name, value in weights.items():
            if value < 0:
                raise ValueError(f"{name} must not be negative")


# 将任意角度折算到 [-pi, pi) 区间，便于计算航向差和角度误差。
def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# 根据左右轮线速度和差速运动学模型，预测一个时间步后的机器人位姿。
def propagate(
    pose: Pose2D,
    left_speed: float,
    right_speed: float,
    *,
    dt: float,
    wheel_base: float,
) -> Pose2D:
    """Advance the differential-drive kinematic model by one time step."""

    linear_speed = (left_speed + right_speed) / 2.0
    angular_speed = (right_speed - left_speed) / wheel_base
    midpoint_yaw = pose.yaw + angular_speed * dt / 2.0
    return Pose2D(
        x=pose.x + linear_speed * math.cos(midpoint_yaw) * dt,
        y=pose.y + linear_speed * math.sin(midpoint_yaw) * dt,
        yaw=normalize_angle(pose.yaw + angular_speed * dt),
    )


class DifferentialDriveMPC:
    """Finite-horizon path tracker with wheel speed and acceleration limits."""

    # 初始化 MPC 控制器，并保存上一周期轮速和上一次解作为 warm-start 状态。
    def __init__(self, config: MPCConfig | None = None) -> None:
        self.config = config or MPCConfig()
        self._previous_wheels = (0.0, 0.0)
        self._last_solution: list[tuple[float, float]] | None = None

    # 开始新路径前重置 warm-start；可选地用上一条真实命令作为轮速基准。
    def reset(self, previous_command: WheelCommand | None = None) -> None:
        """Clear warm-start state before beginning a new path."""

        if previous_command is None:
            self._previous_wheels = (0.0, 0.0)
        else:
            self._previous_wheels = (
                previous_command.left_speed,
                previous_command.right_speed,
            )
        self._last_solution = None

    # 执行一次 MPC 求解：整理参考轨迹、优化未来轮速序列，并返回第一条命令。
    def solve(
        self,
        current_pose: Pose2D,
        reference_poses: Sequence[Pose2D],
    ) -> MPCResult:
        """Solve one MPC step and return the first wheel command.

        ``reference_poses[0]`` represents the reference at the current time.
        Short references are padded with their final pose; long references are
        truncated to ``horizon + 1`` poses.
        """

        reference = self._prepare_reference(reference_poses)
        controls = self._initial_controls(reference)
        controls = self._project_controls(controls)
        cost = self._cost(current_pose, reference, controls)
        converged = False
        completed_iterations = 0

        for iteration in range(1, self.config.optimizer_iterations + 1):
            gradient = self._finite_difference_gradient(
                current_pose,
                reference,
                controls,
                cost,
            )
            gradient = _limit_vector_norm(gradient, maximum_norm=25.0)
            next_controls, next_cost = self._line_search(
                current_pose,
                reference,
                controls,
                gradient,
                cost,
            )
            completed_iterations = iteration

            improvement = cost - next_cost
            if improvement <= self.config.convergence_tolerance:
                converged = True
                break

            controls = next_controls
            cost = next_cost

        predicted_poses = tuple(self._rollout(current_pose, controls))
        left_speed, right_speed = controls[0]
        command = self._make_command(left_speed, right_speed)
        self._previous_wheels = (left_speed, right_speed)
        self._last_solution = controls
        return MPCResult(
            command=command,
            predicted_poses=predicted_poses,
            cost=cost,
            iterations=completed_iterations,
            converged=converged,
        )

    # 将参考轨迹整理成 horizon + 1 个位姿，过长截断、过短用终点补齐。
    def _prepare_reference(
        self,
        reference_poses: Sequence[Pose2D],
    ) -> list[Pose2D]:
        if not reference_poses:
            raise ValueError("reference_poses must contain at least one pose")

        required_size = self.config.horizon + 1
        reference = list(reference_poses[:required_size])
        reference.extend([reference[-1]] * (required_size - len(reference)))
        return reference

    # 根据参考轨迹相邻位姿估计初始轮速；若有上次解则平移复用作为 warm start。
    def _initial_controls(
        self,
        reference: Sequence[Pose2D],
    ) -> list[tuple[float, float]]:
        if self._last_solution is not None:
            return self._last_solution[1:] + [self._last_solution[-1]]

        controls: list[tuple[float, float]] = []
        for start, end in zip(reference, reference[1:]):
            velocity_x = (end.x - start.x) / self.config.dt
            velocity_y = (end.y - start.y) / self.config.dt
            linear_speed = (
                velocity_x * math.cos(start.yaw)
                + velocity_y * math.sin(start.yaw)
            )
            angular_speed = normalize_angle(end.yaw - start.yaw) / self.config.dt
            half_difference = angular_speed * self.config.wheel_base / 2.0
            controls.append(
                (linear_speed - half_difference, linear_speed + half_difference)
            )
        return controls

    # 将候选轮速投影到速度和加速度约束内，保证控制序列物理可执行。
    def _project_controls(
        self,
        controls: Sequence[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        projected: list[tuple[float, float]] = []
        previous_left, previous_right = self._previous_wheels
        maximum_change = self.config.max_wheel_acceleration * self.config.dt

        for left_speed, right_speed in controls:
            left_speed = _clamp(
                left_speed,
                previous_left - maximum_change,
                previous_left + maximum_change,
            )
            right_speed = _clamp(
                right_speed,
                previous_right - maximum_change,
                previous_right + maximum_change,
            )
            left_speed = _clamp(
                left_speed,
                -self.config.max_wheel_speed,
                self.config.max_wheel_speed,
            )
            right_speed = _clamp(
                right_speed,
                -self.config.max_wheel_speed,
                self.config.max_wheel_speed,
            )
            projected.append((left_speed, right_speed))
            previous_left, previous_right = left_speed, right_speed
        return projected

    # 从当前位姿开始按候选轮速序列向前仿真，得到有限时域预测轨迹。
    def _rollout(
        self,
        initial_pose: Pose2D,
        controls: Sequence[tuple[float, float]],
    ) -> list[Pose2D]:
        poses = [initial_pose]
        for left_speed, right_speed in controls:
            poses.append(
                propagate(
                    poses[-1],
                    left_speed,
                    right_speed,
                    dt=self.config.dt,
                    wheel_base=self.config.wheel_base,
                )
            )
        return poses

    # 计算候选控制序列总代价：轨迹误差、航向误差、轮速大小和轮速变化。
    def _cost(
        self,
        current_pose: Pose2D,
        reference: Sequence[Pose2D],
        controls: Sequence[tuple[float, float]],
    ) -> float:
        predicted = self._rollout(current_pose, controls)
        total = 0.0
        previous_left, previous_right = self._previous_wheels

        for index, ((left_speed, right_speed), pose, target) in enumerate(
            zip(controls, predicted[1:], reference[1:])
        ):
            terminal = index == self.config.horizon - 1
            position_weight = (
                self.config.terminal_position_weight
                if terminal
                else self.config.position_weight
            )
            yaw_weight = (
                self.config.terminal_yaw_weight
                if terminal
                else self.config.yaw_weight
            )
            position_error = (pose.x - target.x) ** 2 + (pose.y - target.y) ** 2
            yaw_error = normalize_angle(pose.yaw - target.yaw)
            total += position_weight * position_error
            total += yaw_weight * yaw_error**2
            total += self.config.wheel_speed_weight * (
                left_speed**2 + right_speed**2
            )
            total += self.config.wheel_change_weight * (
                (left_speed - previous_left) ** 2
                + (right_speed - previous_right) ** 2
            )
            previous_left, previous_right = left_speed, right_speed
        return total

    # 对每个轮速变量做有限差分扰动，估计代价关于控制序列的梯度。
    def _finite_difference_gradient(
        self,
        current_pose: Pose2D,
        reference: Sequence[Pose2D],
        controls: list[tuple[float, float]],
        base_cost: float,
    ) -> list[float]:
        epsilon = self.config.finite_difference_epsilon
        gradient: list[float] = []
        flat_controls = _flatten(controls)

        for index, value in enumerate(flat_controls):
            perturbed = flat_controls.copy()
            perturbed[index] = value + epsilon
            projected = self._project_controls(_unflatten(perturbed))
            actual_change = _flatten(projected)[index] - value

            if abs(actual_change) < epsilon * 0.1:
                perturbed[index] = value - epsilon
                projected = self._project_controls(_unflatten(perturbed))
                actual_change = _flatten(projected)[index] - value

            if abs(actual_change) < epsilon * 0.1:
                gradient.append(0.0)
                continue

            perturbed_cost = self._cost(current_pose, reference, projected)
            gradient.append((perturbed_cost - base_cost) / actual_change)
        return gradient

    # 沿负梯度方向尝试不同步长，寻找能降低代价且满足约束的新控制序列。
    def _line_search(
        self,
        current_pose: Pose2D,
        reference: Sequence[Pose2D],
        controls: list[tuple[float, float]],
        gradient: Sequence[float],
        base_cost: float,
    ) -> tuple[list[tuple[float, float]], float]:
        flat_controls = _flatten(controls)
        step_size = self.config.learning_rate

        for _ in range(8):
            candidate = [
                value - step_size * derivative
                for value, derivative in zip(flat_controls, gradient)
            ]
            projected = self._project_controls(_unflatten(candidate))
            candidate_cost = self._cost(current_pose, reference, projected)
            if candidate_cost < base_cost:
                return projected, candidate_cost
            step_size *= 0.5
        return controls, base_cost

    # 将第一组左右轮速度转换成底盘命令，同时计算等效线速度和角速度。
    def _make_command(self, left_speed: float, right_speed: float) -> WheelCommand:
        return WheelCommand(
            left_speed=left_speed,
            right_speed=right_speed,
            linear_speed=(left_speed + right_speed) / 2.0,
            angular_speed=(right_speed - left_speed) / self.config.wheel_base,
        )


# 将 [(left, right), ...] 控制序列展平成一维列表，便于梯度优化。
def _flatten(controls: Sequence[tuple[float, float]]) -> list[float]:
    return [speed for pair in controls for speed in pair]


# 将一维轮速列表还原成左右轮速度对。
def _unflatten(values: Sequence[float]) -> list[tuple[float, float]]:
    return [(values[index], values[index + 1]) for index in range(0, len(values), 2)]


# 限制梯度向量范数，避免单次优化步过大导致线搜索不稳定。
def _limit_vector_norm(values: Sequence[float], maximum_norm: float) -> list[float]:
    norm = math.sqrt(sum(value**2 for value in values))
    if norm <= maximum_norm or norm == 0.0:
        return list(values)
    scale = maximum_norm / norm
    return [value * scale for value in values]


# 将数值裁剪到给定闭区间内，用于速度和加速度约束投影。
def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
