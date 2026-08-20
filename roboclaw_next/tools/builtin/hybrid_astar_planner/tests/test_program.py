from __future__ import annotations

import tempfile
import textwrap
import unittest
from pathlib import Path
from unittest.mock import patch

from roboclaw_next.tools.builtin.hybrid_astar_planner.program import (
    HybridAStarPlannerRunner,
    _decode_plan,
)


class HybridAStarPlannerRunnerTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary_directory.cleanup)
        self._root = Path(self._temporary_directory.name)
        self._map_path = self._root / "map.png"
        self._plan_output_path = self._root / "latest_hybrid_astar_path.json"
        self._map_path.write_bytes(b"test-map")

    async def test_plan_passes_six_pose_values_and_decodes_result(self) -> None:
        executable = self._write_executable(
            """
            import json
            import sys

            args = dict(zip(sys.argv[1::2], sys.argv[2::2]))
            waypoints = [
                {
                    "x": float(args["--start-x"]),
                    "y": float(args["--start-y"]),
                    "yaw": float(args["--start-yaw"]),
                },
                {
                    "x": float(args["--goal-x"]),
                    "y": float(args["--goal-y"]),
                    "yaw": float(args["--goal-yaw"]),
                },
            ]
            print(json.dumps({
                "success": True,
                "frame_id": "map",
                "waypoints": waypoints,
                "waypoint_count": len(waypoints),
                "map_path": args["--map-path"],
                "planning_time_ms": 12.5,
                "message": "Path planned successfully.",
            }))
            """
        )
        runner = HybridAStarPlannerRunner(
            executable_path=executable,
            map_path=self._map_path,
            plan_output_path=self._plan_output_path,
            timeout_sec=1.0,
        )

        result = await runner.plan(1.0, 2.0, 0.25, 8.0, 9.0, -0.5)

        self.assertTrue(result.success)
        self.assertEqual(result.waypoint_count, 2)
        self.assertEqual(result.waypoints[0].x, 1.0)
        self.assertEqual(result.waypoints[-1].yaw, -0.5)
        self.assertEqual(result.map_path, str(self._map_path.resolve()))
        self.assertEqual(result.path_file, str(self._plan_output_path.resolve()))
        self.assertTrue(self._plan_output_path.is_file())
        self.assertIn('"success": true', self._plan_output_path.read_text())

    async def test_domain_failure_is_returned_even_with_nonzero_exit_code(self) -> None:
        executable = self._write_executable(
            """
            import json
            import sys

            print(json.dumps({
                "success": False,
                "frame_id": "map",
                "waypoints": [],
                "waypoint_count": 0,
                "map_path": sys.argv[-1],
                "planning_time_ms": 1.0,
                "message": "No collision-free path was found.",
            }))
            raise SystemExit(2)
            """
        )
        runner = HybridAStarPlannerRunner(
            executable_path=executable,
            map_path=self._map_path,
            plan_output_path=self._plan_output_path,
        )

        result = await runner.plan(1.0, 2.0, 0.0, 8.0, 9.0, 0.0)

        self.assertFalse(result.success)
        self.assertEqual(result.waypoints, ())
        self.assertIn("No collision-free path", result.message)
        self.assertIn('"success": false', self._plan_output_path.read_text())

    async def test_invalid_program_output_raises_runtime_error(self) -> None:
        executable = self._write_executable("print('not-json')")
        runner = HybridAStarPlannerRunner(
            executable_path=executable,
            map_path=self._map_path,
            plan_output_path=self._plan_output_path,
        )

        with self.assertRaisesRegex(RuntimeError, "invalid result JSON"):
            await runner.plan(1.0, 2.0, 0.0, 8.0, 9.0, 0.0)
        self.assertIn('"success": false', self._plan_output_path.read_text())

    async def test_non_finite_input_is_rejected_before_process_start(self) -> None:
        runner = HybridAStarPlannerRunner(
            executable_path=self._root / "unused",
            map_path=self._map_path,
            plan_output_path=self._plan_output_path,
        )

        with self.assertRaisesRegex(ValueError, "must be finite"):
            await runner.plan(float("nan"), 2.0, 0.0, 8.0, 9.0, 0.0)

    def test_uses_project_default_when_no_executable_is_injected(self) -> None:
        executable = self._write_executable("raise SystemExit(0)")
        with patch(
            "roboclaw_next.tools.builtin.hybrid_astar_planner.program."
            "DEFAULT_EXECUTABLE_PATH",
            executable,
        ):
            runner = HybridAStarPlannerRunner(map_path=self._map_path)
            self.assertEqual(runner._resolve_executable(), executable.resolve())

    def _write_executable(self, body: str) -> Path:
        executable = self._root / "hybrid_astar_plan"
        executable.write_text(
            "#!/usr/bin/env python3\n" + textwrap.dedent(body).lstrip(),
            encoding="utf-8",
        )
        executable.chmod(0o755)
        return executable


class DecodePlanTest(unittest.TestCase):
    def test_waypoint_count_must_match_array(self) -> None:
        payload = b"""{
            "success": false,
            "frame_id": "map",
            "waypoints": [],
            "waypoint_count": 1,
            "map_path": "/tmp/map.png",
            "planning_time_ms": 0,
            "message": "failed"
        }"""

        with self.assertRaisesRegex(ValueError, "does not match"):
            _decode_plan(payload)

    def test_successful_plan_must_contain_waypoints(self) -> None:
        payload = b"""{
            "success": true,
            "frame_id": "map",
            "waypoints": [],
            "waypoint_count": 0,
            "map_path": "/tmp/map.png",
            "planning_time_ms": 0,
            "message": "planned"
        }"""

        with self.assertRaisesRegex(ValueError, "at least one waypoint"):
            _decode_plan(payload)

    def test_waypoint_values_must_be_finite(self) -> None:
        payload = b"""{
            "success": true,
            "frame_id": "map",
            "waypoints": [{"x": NaN, "y": 2, "yaw": 0}],
            "waypoint_count": 1,
            "map_path": "/tmp/map.png",
            "planning_time_ms": 1,
            "message": "planned"
        }"""

        with self.assertRaisesRegex(ValueError, "finite number"):
            _decode_plan(payload)


if __name__ == "__main__":
    unittest.main()
