"""Load Hybrid A* paths and prepare B-Spline reference poses for MPC."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from .controller import Pose2D, normalize_angle


DEFAULT_REFERENCE_LINEAR_SPEED = 0.55
DEFAULT_BSPLINE_TABLE_SUBDIVISIONS = 8


@dataclass(frozen=True)
class PathPoint:
    """One planar path point from Hybrid A*."""

    x: float
    y: float
    direction: str


@dataclass(frozen=True)
class ReferencePath:
    """A B-Spline reference path prepared for MPC tracking."""

    poses: tuple[Pose2D, ...]
    raw_point_count: int
    source: str


@dataclass(frozen=True)
class _PoseSample:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class _SplineState:
    x: float
    y: float
    dx: float
    dy: float
    distance: float


def load_hybrid_astar_reference_path(
    path_file: str | Path,
    *,
    step_distance: float,
) -> ReferencePath:
    """Load a Hybrid A* JSON path and convert it to B-Spline poses."""

    if step_distance <= 0:
        raise ValueError("step_distance must be greater than zero")

    path = Path(path_file).expanduser().resolve()
    raw_points = load_hybrid_astar_path_points(path)
    poses = _build_bspline_reference_poses(
        raw_points,
        step_distance=step_distance,
    )
    return ReferencePath(
        poses=tuple(poses),
        raw_point_count=len(raw_points),
        source=(
            f"{path}: {len(raw_points)} Hybrid A* path points, "
            f"{len(poses)} B-Spline MPC poses"
        ),
    )


def load_hybrid_astar_path_points(path_file: str | Path) -> tuple[PathPoint, ...]:
    """Load and validate planar path points from a Hybrid A* result JSON."""

    path = Path(path_file).expanduser().resolve()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"reference_path_file is not readable JSON: {path}") from exc

    if not isinstance(payload, dict):
        raise ValueError(f"reference_path_file must contain a JSON object: {path}")
    if payload.get("success") is not True:
        message = payload.get("message") or "plan is not successful"
        raise ValueError(
            "reference_path_file does not contain a successful Hybrid A* "
            f"path: {message}"
        )

    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError(f"reference_path_file does not contain path points: {path}")
    waypoint_count = payload.get("waypoint_count")
    if waypoint_count is not None and waypoint_count != len(raw_waypoints):
        raise ValueError(
            "reference_path_file waypoint_count does not match path points: "
            f"{path}"
        )

    points: list[PathPoint] = []
    for index, raw_waypoint in enumerate(raw_waypoints):
        if not isinstance(raw_waypoint, dict):
            raise ValueError(f"path point {index} must be a JSON object")
        points.append(_decode_planar_point(raw_waypoint, index))
    return tuple(points)


def _build_bspline_reference_poses(
    points: Sequence[PathPoint],
    *,
    step_distance: float,
) -> list[Pose2D]:
    if not points:
        raise ValueError("path must contain at least one point")

    points = _drop_repeated_points(tuple(points))
    if len(points) == 1:
        return [
            Pose2D(
                x=points[0].x,
                y=points[0].y,
                yaw=_direction_yaw(0.0, points[0].direction),
            )
        ]

    poses: list[Pose2D] = []
    for segment in _split_by_direction(points):
        segment = _drop_repeated_points(segment)
        if len(segment) == 1:
            if not poses:
                poses.append(
                    Pose2D(
                        x=segment[0].x,
                        y=segment[0].y,
                        yaw=_direction_yaw(0.0, segment[0].direction),
                    )
                )
            continue

        samples = _sample_bspline_segment(
            segment,
            step_distance=step_distance,
            direction=segment[0].direction,
        )
        if poses and samples and _same_position(poses[-1], samples[0]):
            samples = samples[1:]
        poses.extend(
            Pose2D(x=sample.x, y=sample.y, yaw=sample.yaw)
            for sample in samples
        )

    if not poses:
        raise ValueError("path did not produce a reference path")
    return poses


def _sample_bspline_segment(
    points: Sequence[PathPoint],
    *,
    step_distance: float,
    direction: str,
) -> list[_PoseSample]:
    table = _build_bspline_arc_table(points)
    if len(table) == 1:
        point = points[0]
        return [
            _PoseSample(
                x=point.x,
                y=point.y,
                yaw=_direction_yaw(0.0, direction),
            )
        ]

    total_distance = table[-1].distance
    if total_distance <= 0:
        point = points[0]
        return [
            _PoseSample(
                x=point.x,
                y=point.y,
                yaw=_direction_yaw(0.0, direction),
            )
        ]

    sample_distances = _sample_distances(total_distance, step_distance)
    samples: list[_PoseSample] = []
    table_index = 0
    last_tangent_yaw = _state_tangent_yaw(table[0], table[1])

    for sample_distance in sample_distances:
        while (
            table_index < len(table) - 2
            and table[table_index + 1].distance < sample_distance
        ):
            table_index += 1

        left = table[table_index]
        right = table[table_index + 1]
        span = right.distance - left.distance
        ratio = 0.0 if span <= 0 else (sample_distance - left.distance) / span
        state = _interpolate_state(left, right, ratio)
        tangent_yaw = _state_tangent_yaw(state, right, fallback=last_tangent_yaw)
        last_tangent_yaw = tangent_yaw
        samples.append(
            _PoseSample(
                x=state.x,
                y=state.y,
                yaw=_direction_yaw(tangent_yaw, direction),
            )
        )
    return samples


def _split_by_direction(
    points: tuple[PathPoint, ...],
) -> list[tuple[PathPoint, ...]]:
    segments: list[tuple[PathPoint, ...]] = []
    current_direction = points[0].direction
    current_segment = [_with_direction(points[0], current_direction)]

    for point in points[1:]:
        if point.direction == current_direction:
            current_segment.append(point)
            continue

        # 换向点同时作为前一段终点和后一段起点，避免样条跨越 cusp。
        current_segment.append(_with_direction(point, current_direction))
        segments.append(tuple(current_segment))
        current_direction = point.direction
        current_segment = [point]

    segments.append(tuple(current_segment))
    return segments


def _with_direction(point: PathPoint, direction: str) -> PathPoint:
    return PathPoint(x=point.x, y=point.y, direction=direction)


def _same_position(left: Any, right: Any) -> bool:
    return math.hypot(left.x - right.x, left.y - right.y) <= 1e-9


def _sample_distances(total_distance: float, step_distance: float) -> list[float]:
    distances = [0.0]
    next_distance = step_distance
    while next_distance < total_distance:
        distances.append(next_distance)
        next_distance += step_distance
    if not math.isclose(distances[-1], total_distance):
        distances.append(total_distance)
    return distances


def _build_bspline_arc_table(points: Sequence[PathPoint]) -> list[_SplineState]:
    if len(points) < 3:
        return _build_linear_arc_table(points)

    controls = (
        [points[0], points[0], points[0]]
        + list(points[1:-1])
        + [points[-1], points[-1], points[-1]]
    )
    raw_states: list[_SplineState] = []
    piece_count = len(controls) - 3
    for piece_index in range(piece_count):
        for subdivision in range(DEFAULT_BSPLINE_TABLE_SUBDIVISIONS):
            if piece_index > 0 and subdivision == 0:
                continue
            u = subdivision / DEFAULT_BSPLINE_TABLE_SUBDIVISIONS
            raw_states.append(
                _evaluate_uniform_cubic_bspline(controls, piece_index, u)
            )
    raw_states.append(
        _evaluate_uniform_cubic_bspline(controls, piece_count - 1, 1.0)
    )
    return _attach_arc_lengths(raw_states)


def _build_linear_arc_table(points: Sequence[PathPoint]) -> list[_SplineState]:
    if not points:
        return []
    if len(points) == 1:
        return [
            _SplineState(
                x=points[0].x,
                y=points[0].y,
                dx=0.0,
                dy=0.0,
                distance=0.0,
            )
        ]

    raw_states: list[_SplineState] = []
    for index, point in enumerate(points):
        if index == len(points) - 1:
            previous = points[index - 1]
            dx = point.x - previous.x
            dy = point.y - previous.y
        else:
            following = points[index + 1]
            dx = following.x - point.x
            dy = following.y - point.y
        raw_states.append(
            _SplineState(
                x=point.x,
                y=point.y,
                dx=dx,
                dy=dy,
                distance=0.0,
            )
        )
    return _attach_arc_lengths(raw_states)


def _evaluate_uniform_cubic_bspline(
    controls: Sequence[PathPoint],
    piece_index: int,
    u: float,
) -> _SplineState:
    p0, p1, p2, p3 = controls[piece_index : piece_index + 4]
    u2 = u * u
    u3 = u2 * u

    basis0 = (-u3 + 3.0 * u2 - 3.0 * u + 1.0) / 6.0
    basis1 = (3.0 * u3 - 6.0 * u2 + 4.0) / 6.0
    basis2 = (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0) / 6.0
    basis3 = u3 / 6.0

    derivative0 = (-3.0 * u2 + 6.0 * u - 3.0) / 6.0
    derivative1 = (9.0 * u2 - 12.0 * u) / 6.0
    derivative2 = (-9.0 * u2 + 6.0 * u + 3.0) / 6.0
    derivative3 = 3.0 * u2 / 6.0

    return _SplineState(
        x=basis0 * p0.x + basis1 * p1.x + basis2 * p2.x + basis3 * p3.x,
        y=basis0 * p0.y + basis1 * p1.y + basis2 * p2.y + basis3 * p3.y,
        dx=(
            derivative0 * p0.x
            + derivative1 * p1.x
            + derivative2 * p2.x
            + derivative3 * p3.x
        ),
        dy=(
            derivative0 * p0.y
            + derivative1 * p1.y
            + derivative2 * p2.y
            + derivative3 * p3.y
        ),
        distance=0.0,
    )


def _attach_arc_lengths(states: Sequence[_SplineState]) -> list[_SplineState]:
    if not states:
        return []

    table: list[_SplineState] = []
    distance = 0.0
    previous = states[0]
    table.append(_replace_distance(previous, distance))
    for state in states[1:]:
        distance += math.hypot(state.x - previous.x, state.y - previous.y)
        table.append(_replace_distance(state, distance))
        previous = state
    return table


def _replace_distance(state: _SplineState, distance: float) -> _SplineState:
    return _SplineState(
        x=state.x,
        y=state.y,
        dx=state.dx,
        dy=state.dy,
        distance=distance,
    )


def _interpolate_state(
    left: _SplineState,
    right: _SplineState,
    ratio: float,
) -> _SplineState:
    return _SplineState(
        x=_lerp(left.x, right.x, ratio),
        y=_lerp(left.y, right.y, ratio),
        dx=_lerp(left.dx, right.dx, ratio),
        dy=_lerp(left.dy, right.dy, ratio),
        distance=_lerp(left.distance, right.distance, ratio),
    )


def _state_tangent_yaw(
    state: _SplineState,
    next_state: _SplineState | None = None,
    *,
    fallback: float = 0.0,
) -> float:
    if math.hypot(state.dx, state.dy) > 1e-9:
        return math.atan2(state.dy, state.dx)
    if next_state is not None:
        dx = next_state.x - state.x
        dy = next_state.y - state.y
        if math.hypot(dx, dy) > 1e-9:
            return math.atan2(dy, dx)
    return fallback


def _lerp(left: float, right: float, ratio: float) -> float:
    return left + (right - left) * ratio


def _decode_planar_point(raw_waypoint: dict[str, Any], index: int) -> PathPoint:
    try:
        x = float(raw_waypoint["x"])
        y = float(raw_waypoint["y"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"path point {index} must contain numeric x and y") from error

    direction = raw_waypoint.get("direction")
    if direction not in {"forward", "reverse"}:
        raise ValueError(
            f"path point {index} must contain direction 'forward' or 'reverse'"
        )

    if not all(math.isfinite(value) for value in (x, y)):
        raise ValueError(f"path point {index} contains a non-finite value")
    return PathPoint(x=x, y=y, direction=direction)


def _drop_repeated_points(points: tuple[PathPoint, ...]) -> tuple[PathPoint, ...]:
    kept = [points[0]]
    for point in points[1:]:
        previous = kept[-1]
        if math.hypot(point.x - previous.x, point.y - previous.y) > 1e-9:
            kept.append(point)
    return tuple(kept)


def _direction_yaw(path_tangent_yaw: float, direction: str) -> float:
    if direction == "reverse":
        return normalize_angle(path_tangent_yaw + math.pi)
    return normalize_angle(path_tangent_yaw)
