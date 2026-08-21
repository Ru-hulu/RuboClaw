from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from roboclaw_next.tools.builtin.path_tracking import program
from roboclaw_next.tools.builtin.path_tracking.program import (
    PathTrackingProcessManager,
)


class PathTrackingProcessManagerTest(unittest.TestCase):
    def test_missing_reference_path_fails_without_default_path(self) -> None:
        manager = PathTrackingProcessManager(localization_manager=object())  # type: ignore[arg-type]

        with tempfile.TemporaryDirectory() as temporary_directory:
            original_latest_path = program.DEFAULT_PLAN_OUTPUT_PATH
            program.DEFAULT_PLAN_OUTPUT_PATH = (
                Path(temporary_directory) / "missing_latest_path.json"
            )
            try:
                with self.assertRaisesRegex(ValueError, "no Hybrid A"):
                    manager._resolve_reference_path_file(None, Path(temporary_directory))
            finally:
                program.DEFAULT_PLAN_OUTPUT_PATH = original_latest_path

    def test_latest_reference_path_is_used_when_available(self) -> None:
        payload = {
            "success": True,
            "frame_id": "map",
            "waypoints": [
                {"x": 0.0, "y": 0.0, "direction": "forward"},
                {"x": 1.0, "y": 0.0, "direction": "forward"},
            ],
            "waypoint_count": 2,
            "map_path": "test.png",
            "planning_time_ms": 1.0,
            "message": "ok",
        }
        manager = PathTrackingProcessManager(localization_manager=object())  # type: ignore[arg-type]

        with tempfile.TemporaryDirectory() as temporary_directory:
            latest_path = Path(temporary_directory) / "latest_path.json"
            latest_path.write_text(json.dumps(payload), encoding="utf-8")
            original_latest_path = program.DEFAULT_PLAN_OUTPUT_PATH
            program.DEFAULT_PLAN_OUTPUT_PATH = latest_path
            try:
                resolved = manager._resolve_reference_path_file(
                    None,
                    Path(temporary_directory),
                )
            finally:
                program.DEFAULT_PLAN_OUTPUT_PATH = original_latest_path

        self.assertEqual(resolved, latest_path.resolve())


if __name__ == "__main__":
    unittest.main()
