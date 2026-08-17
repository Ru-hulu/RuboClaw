"""Temporary 2D differential-drive simulator with a small HTTP API."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


SIMULATION_DT = 0.05
MAX_WHEEL_SPEED = 1.5
HTML_PATH = Path(__file__).with_name("grid_world.html")


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class GridWorldSimulator:
    """Thread-safe grid world with differential-drive kinematics."""

    def __init__(
        self,
        *,
        columns: int = 16,
        rows: int = 12,
        cell_size: float = 0.5,
        wheel_base: float = 0.45,
    ) -> None:
        self.columns = columns
        self.rows = rows
        self.cell_size = cell_size
        self.wheel_base = wheel_base
        self.robot_length = 0.55
        self.robot_width = 0.35
        self.obstacles = self._default_obstacles()
        self._lock = threading.Lock()
        self._reset_unlocked(Pose2D(x=1.0, y=1.0, yaw=0.0))

    @property
    def width(self) -> float:
        return self.columns * self.cell_size

    @property
    def height(self) -> float:
        return self.rows * self.cell_size

    def _default_obstacles(self) -> set[tuple[int, int]]:
        obstacles = {(row, 7) for row in range(1, 10) if row not in (4, 5)}
        obstacles.update((9, column) for column in range(2, 6))
        obstacles.update((2, column) for column in range(11, 14))
        return obstacles

    def _reset_unlocked(self, pose: Pose2D) -> None:
        self.pose = Pose2D(x=pose.x, y=pose.y, yaw=_normalize_angle(pose.yaw))
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.running = False
        self.collision = False
        self.simulation_time = 0.0
        self.trajectory = [(self.pose.x, self.pose.y)]
        self.target_pose: Pose2D | None = None
        self.planned_path: list[Pose2D] = []

    def reset(self, pose: Pose2D | None = None) -> None:
        next_pose = pose or Pose2D(x=1.0, y=1.0, yaw=0.0)
        with self._lock:
            if self._is_blocked(next_pose.x, next_pose.y):
                raise ValueError("Reset pose must be inside a free grid cell")
            self._reset_unlocked(next_pose)

    def clear_trajectory(self) -> None:
        with self._lock:
            self.trajectory = [(self.pose.x, self.pose.y)]

    def set_running(self, running: bool) -> None:
        with self._lock:
            self.running = running
            if running:
                self.collision = False

    def set_wheel_speeds(self, left_speed: float, right_speed: float) -> None:
        with self._lock:
            self.left_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left_speed))
            self.right_speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right_speed))
            self.collision = False

    def set_navigation_plan(
        self,
        target_pose: Pose2D,
        planned_path: list[Pose2D],
    ) -> None:
        with self._lock:
            if self._is_blocked(target_pose.x, target_pose.y):
                raise ValueError("Target pose must be inside a free grid cell")
            self.target_pose = Pose2D(
                x=target_pose.x,
                y=target_pose.y,
                yaw=_normalize_angle(target_pose.yaw),
            )
            self.planned_path = [
                Pose2D(x=pose.x, y=pose.y, yaw=_normalize_angle(pose.yaw))
                for pose in planned_path
            ]

    def clear_navigation_plan(self) -> None:
        with self._lock:
            self.target_pose = None
            self.planned_path = []

    def step(self, dt: float) -> None:
        with self._lock:
            if not self.running:
                return

            linear_speed = (self.left_speed + self.right_speed) / 2.0
            angular_speed = (self.right_speed - self.left_speed) / self.wheel_base
            midpoint_yaw = self.pose.yaw + angular_speed * dt / 2.0
            next_pose = Pose2D(
                x=self.pose.x + linear_speed * math.cos(midpoint_yaw) * dt,
                y=self.pose.y + linear_speed * math.sin(midpoint_yaw) * dt,
                yaw=_normalize_angle(self.pose.yaw + angular_speed * dt),
            )

            if self._is_blocked(next_pose.x, next_pose.y):
                self.left_speed = 0.0
                self.right_speed = 0.0
                self.running = False
                self.collision = True
                return

            moved = math.hypot(next_pose.x - self.pose.x, next_pose.y - self.pose.y)
            self.pose = next_pose
            self.simulation_time += dt
            if moved >= 0.005:
                self.trajectory.append((self.pose.x, self.pose.y))
                self.trajectory = self.trajectory[-4000:]

    def _is_blocked(self, x: float, y: float) -> bool:
        if not (0.0 <= x < self.width and 0.0 <= y < self.height):
            return True
        column = int(x / self.cell_size)
        row = int(y / self.cell_size)
        return (row, column) in self.obstacles

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "world": {
                    "columns": self.columns,
                    "rows": self.rows,
                    "cell_size": self.cell_size,
                    "width": self.width,
                    "height": self.height,
                    "obstacles": [
                        {"row": row, "column": column}
                        for row, column in sorted(self.obstacles)
                    ],
                },
                "robot": {
                    "x": self.pose.x,
                    "y": self.pose.y,
                    "yaw": self.pose.yaw,
                    "length": self.robot_length,
                    "width": self.robot_width,
                    "wheel_base": self.wheel_base,
                    "left_speed": self.left_speed,
                    "right_speed": self.right_speed,
                    "running": self.running,
                    "collision": self.collision,
                },
                "simulation_time": self.simulation_time,
                "trajectory": [
                    {"x": x, "y": y} for x, y in self.trajectory
                ],
                "target_pose": (
                    {
                        "x": self.target_pose.x,
                        "y": self.target_pose.y,
                        "yaw": self.target_pose.yaw,
                    }
                    if self.target_pose is not None
                    else None
                ),
                "planned_path": [
                    {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
                    for pose in self.planned_path
                ],
            }


class GridWorldRequestHandler(BaseHTTPRequestHandler):
    simulator: GridWorldSimulator

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path in ("/", "/grid_world.html"):
            self._send_html()
            return
        if self.path == "/state":
            self._send_json(self.simulator.snapshot())
            return
        if self.path == "/health":
            self._send_json({"status": "ok"})
            return
        self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        try:
            payload = self._read_json()
            if self.path == "/control":
                left_speed = _required_number(payload, "left_speed")
                right_speed = _required_number(payload, "right_speed")
                self.simulator.set_wheel_speeds(left_speed, right_speed)
            elif self.path == "/run":
                self.simulator.set_running(True)
            elif self.path == "/pause":
                self.simulator.set_running(False)
            elif self.path == "/reset":
                pose = _pose_from_payload(payload) if payload else None
                self.simulator.reset(pose)
            elif self.path == "/clear_trajectory":
                self.simulator.clear_trajectory()
            elif self.path == "/navigation_plan":
                target_payload = payload.get("target")
                if not isinstance(target_payload, dict):
                    raise ValueError("target must be a JSON object")
                path_payload = payload.get("path", [])
                if not isinstance(path_payload, list):
                    raise ValueError("path must be a JSON array")
                target_pose = _pose_from_payload(target_payload)
                planned_path = [
                    _pose_from_payload(path_point)
                    for path_point in path_payload
                    if isinstance(path_point, dict)
                ]
                if len(planned_path) != len(path_payload):
                    raise ValueError("Every path point must be a JSON object")
                self.simulator.set_navigation_plan(target_pose, planned_path)
            elif self.path == "/clear_navigation_plan":
                self.simulator.clear_navigation_plan()
            else:
                self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)
                return
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        self._send_json(self.simulator.snapshot())

    def _read_json(self) -> dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length > 16_384:
            raise ValueError("Request body is too large")
        if content_length == 0:
            return {}
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object")
        return payload

    def _send_html(self) -> None:
        body = HTML_PATH.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(
        self,
        payload: dict[str, Any],
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: Any) -> None:
        return


def _required_number(payload: dict[str, Any], field_name: str) -> float:
    value = payload.get(field_name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be finite")
    return number


def _pose_from_payload(payload: dict[str, Any]) -> Pose2D:
    return Pose2D(
        x=_required_number(payload, "x"),
        y=_required_number(payload, "y"),
        yaw=_required_number(payload, "yaw"),
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _simulation_loop(
    simulator: GridWorldSimulator,
    stop_event: threading.Event,
) -> None:
    next_step = time.monotonic()
    while not stop_event.is_set():
        next_step += SIMULATION_DT
        simulator.step(SIMULATION_DT)
        stop_event.wait(max(0.0, next_step - time.monotonic()))


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the temporary grid-world simulator.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    simulator = GridWorldSimulator()
    GridWorldRequestHandler.simulator = simulator
    server = ThreadingHTTPServer(
        (arguments.host, arguments.port),
        GridWorldRequestHandler,
    )
    server.daemon_threads = True
    stop_event = threading.Event()
    simulation_thread = threading.Thread(
        target=_simulation_loop,
        args=(simulator, stop_event),
        daemon=True,
    )
    simulation_thread.start()

    print(f"Grid-world simulator: http://{arguments.host}:{arguments.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.server_close()
        simulation_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
