#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "planner.h"

#ifndef HYBRID_ASTAR_DEFAULT_MAP_PATH
#define HYBRID_ASTAR_DEFAULT_MAP_PATH "maps/empty_80x80.png"
#endif

namespace {

struct Options {
  std::optional<double> startX;
  std::optional<double> startY;
  std::optional<double> startYaw;
  std::optional<double> goalX;
  std::optional<double> goalY;
  std::optional<double> goalYaw;
  std::string mapPath = HYBRID_ASTAR_DEFAULT_MAP_PATH;
};

double parseNumber(const std::string& raw, const std::string& option) {
  std::size_t parsed = 0;
  const double value = std::stod(raw, &parsed);
  if (parsed != raw.size() || !std::isfinite(value)) {
    throw std::invalid_argument(option + " must be a finite number.");
  }
  return value;
}

Options parseArguments(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument("Missing value for " + option + ".");
    }
    const std::string value = argv[++index];

    if (option == "--start-x") {
      options.startX = parseNumber(value, option);
    } else if (option == "--start-y") {
      options.startY = parseNumber(value, option);
    } else if (option == "--start-yaw") {
      options.startYaw = parseNumber(value, option);
    } else if (option == "--goal-x") {
      options.goalX = parseNumber(value, option);
    } else if (option == "--goal-y") {
      options.goalY = parseNumber(value, option);
    } else if (option == "--goal-yaw") {
      options.goalYaw = parseNumber(value, option);
    } else if (option == "--map-path") {
      options.mapPath = value;
    } else {
      throw std::invalid_argument("Unknown option: " + option + ".");
    }
  }

  if (!options.startX || !options.startY || !options.startYaw ||
      !options.goalX || !options.goalY || !options.goalYaw) {
    throw std::invalid_argument(
        "Required options: --start-x --start-y --start-yaw "
        "--goal-x --goal-y --goal-yaw.");
  }
  return options;
}

std::string escapeJson(const std::string& value) {
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(character) << std::dec;
        } else {
          escaped << character;
        }
    }
  }
  return escaped.str();
}

std::string directionToJson(HybridAStar::MotionDirection direction) {
  return direction == HybridAStar::MotionDirection::Reverse ? "reverse"
                                                            : "forward";
}

void writeResult(
    const HybridAStar::PlanResult& result,
    const std::string& mapPath) {
  std::cout << std::setprecision(15);
  std::cout << "{\"success\":" << (result.success ? "true" : "false")
            << ",\"frame_id\":\"map\",\"waypoints\":[";
  for (std::size_t index = 0; index < result.waypoints.size(); ++index) {
    if (index > 0) {
      std::cout << ',';
    }
    const auto& waypoint = result.waypoints[index];
    std::cout << "{\"x\":" << waypoint.x << ",\"y\":" << waypoint.y
              << ",\"direction\":\"" << directionToJson(waypoint.direction)
              << "\"}";
  }
  std::cout << "],\"waypoint_count\":" << result.waypoints.size()
            << ",\"map_path\":\"" << escapeJson(mapPath) << "\""
            << ",\"planning_time_ms\":" << result.planningTimeMs
            << ",\"message\":\"" << escapeJson(result.message) << "\"}\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string mapPath = HYBRID_ASTAR_DEFAULT_MAP_PATH;
  try {
    const Options options = parseArguments(argc, argv);
    mapPath = std::filesystem::absolute(options.mapPath).lexically_normal().string();

    HybridAStar::Planner planner(
        HybridAStar::Planner::loadMapFromImage(mapPath));
    const HybridAStar::PlanResult result = planner.plan(
        HybridAStar::Pose2D{*options.startX, *options.startY, *options.startYaw},
        HybridAStar::Pose2D{*options.goalX, *options.goalY, *options.goalYaw});
    writeResult(result, mapPath);
    return result.success ? 0 : 2;
  } catch (const std::exception& error) {
    HybridAStar::PlanResult result;
    result.message = error.what();
    writeResult(result, mapPath);
    return 1;
  }
}
