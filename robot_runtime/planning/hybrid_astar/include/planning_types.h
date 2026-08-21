#ifndef HYBRID_ASTAR_PLANNING_TYPES_H
#define HYBRID_ASTAR_PLANNING_TYPES_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace HybridAStar {

enum class MotionDirection {
  Forward,
  Reverse,
};

// 与通信框架无关的二维位姿，位置单位为米，偏航角单位为弧度。
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  MotionDirection direction = MotionDirection::Forward;
};

// Hybrid A* 使用的二值栅格地图；0 表示可通行，非 0 表示占用。
struct GridMap {
  unsigned int width = 0;
  unsigned int height = 0;
  double resolution = 1.0;
  std::vector<std::uint8_t> data;

  bool isValid() const {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == static_cast<std::size_t>(width) * height;
  }
};

// 规划核心的统一返回值，可由 CLI、MCP 或其他上层适配器直接转换。
struct PlanResult {
  bool success = false;
  std::vector<Pose2D> waypoints;
  double planningTimeMs = 0.0;
  std::string message;
};

}  // namespace HybridAStar

#endif  // HYBRID_ASTAR_PLANNING_TYPES_H
