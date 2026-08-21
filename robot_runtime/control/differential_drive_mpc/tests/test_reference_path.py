from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from robot_runtime.control.differential_drive_mpc.reference_path import (
    load_hybrid_astar_path_points,
    load_hybrid_astar_reference_path,
)


class ReferencePathTest(unittest.TestCase):
    def test_loader_returns_bspline_reference_poses(self) -> None:
        payload = {
            "success": True,
            "frame_id": "map",
            "waypoints": [
                {"x": 0.0, "y": 0.0, "direction": "forward"},
                {"x": 1.0, "y": 0.0, "direction": "forward"},
                {"x": 2.0, "y": 0.5, "direction": "forward"},
                {"x": 3.0, "y": 0.5, "direction": "forward"},
            ],
            "waypoint_count": 4,
            "map_path": "test.png",
            "planning_time_ms": 1.0,
            "message": "ok",
        }

        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "plan.json"
            path.write_text(json.dumps(payload), encoding="utf-8")

            reference = load_hybrid_astar_reference_path(
                path,
                step_distance=0.2,
            )

        self.assertEqual(reference.raw_point_count, 4)
        self.assertGreater(len(reference.poses), 4)
        self.assertAlmostEqual(reference.poses[0].x, 0.0)
        self.assertAlmostEqual(reference.poses[0].y, 0.0)
        self.assertAlmostEqual(reference.poses[-1].x, 3.0)
        self.assertAlmostEqual(reference.poses[-1].y, 0.5)
        self.assertTrue(all(math.isfinite(pose.yaw) for pose in reference.poses))
        self.assertIn("B-Spline", reference.source)

    def test_reverse_segment_yaw_uses_vehicle_heading(self) -> None:
        payload = {
            "success": True,
            "frame_id": "map",
            "waypoints": [
                {"x": 2.0, "y": 0.0, "direction": "reverse"},
                {"x": 1.0, "y": 0.0, "direction": "reverse"},
                {"x": 0.0, "y": 0.0, "direction": "reverse"},
            ],
            "waypoint_count": 3,
            "map_path": "test.png",
            "planning_time_ms": 1.0,
            "message": "ok",
        }

        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "plan.json"
            path.write_text(json.dumps(payload), encoding="utf-8")

            reference = load_hybrid_astar_reference_path(
                path,
                step_distance=0.25,
            )

        # The path tangent points toward negative x, but a reverse-driving robot
        # should face positive x while backing along that path.
        self.assertAlmostEqual(reference.poses[1].yaw, 0.0, places=6)

    def test_loader_validates_hybrid_astar_contract(self) -> None:
        payload = {
            "success": True,
            "frame_id": "map",
            "waypoints": [{"x": 0.0, "y": 0.0, "direction": "sideways"}],
            "waypoint_count": 1,
        }

        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "plan.json"
            path.write_text(json.dumps(payload), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "forward' or 'reverse"):
                load_hybrid_astar_path_points(path)


if __name__ == "__main__":
    unittest.main()
