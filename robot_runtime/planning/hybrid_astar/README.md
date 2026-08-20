# Hybrid A* Planner

This directory contains RoboClaw's standalone Hybrid A* planner. The planning
core uses plain C++ data types and does not depend on ROS nodes, messages,
services, or process lifecycle management.

The runtime call path is:

```text
MCP Tool -> Python one-shot runner -> hybrid_astar_plan CLI -> C++ planner core
```

## Tool contract

The MCP server exposes one tool:

```text
plan_hybrid_astar_path(
  start_x, start_y, start_yaw,
  goal_x, goal_y, goal_yaw
)
```

Positions are map-frame meters and yaw values are radians. The structured
result contains:

- `success`: whether a collision-free path was found
- `frame_id`: always `map`
- `waypoints`: ordered `{x, y, yaw}` path points
- `waypoint_count`: number of returned waypoints
- `map_path`: PNG map used by the request
- `planning_time_ms`: planner execution time
- `message`: outcome or failure reason

## Fixed map

The tool always loads:

```text
maps/map_demo.png
```

The map uses these conventions:

- `1 pixel = 1 grid cell = 1 meter`
- dark pixels are occupied and light pixels are free
- grayscale values below `128` are occupied
- the image bottom row corresponds to map coordinate `y = 0`

The map path is intentionally absent from the MCP input schema. It remains an
internal runtime configuration rather than an Agent decision.

## Build

Required system packages are CMake, Boost, OMPL, and OpenCV. Build the
standalone executable from the repository root:

```bash
cmake \
  -S robot_runtime/planning/hybrid_astar \
  -B robot_runtime/planning/hybrid_astar/build
cmake --build robot_runtime/planning/hybrid_astar/build --parallel
```

The Python runner discovers the resulting binary at:

```text
robot_runtime/planning/hybrid_astar/build/hybrid_astar_plan
```

## Direct CLI

```bash
robot_runtime/planning/hybrid_astar/build/hybrid_astar_plan \
  --start-x 5 --start-y 5 --start-yaw 0 \
  --goal-x 60 --goal-y 60 --goal-yaw 0 \
  --map-path robot_runtime/planning/hybrid_astar/maps/map_demo.png
```

The CLI writes exactly one JSON object to stdout. Diagnostic messages use
stderr so the MCP adapter can validate stdout as a strict machine interface.
