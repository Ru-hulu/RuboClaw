#include "planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/imgcodecs.hpp>

#include "helper.h"
#include "lookup.h"

using namespace HybridAStar;

namespace {

double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

bool samePose(const Pose2D& left, const Pose2D& right) {
  constexpr double epsilon = 1e-6;
  return std::abs(left.x - right.x) < epsilon &&
         std::abs(left.y - right.y) < epsilon &&
         std::abs(std::remainder(left.yaw - right.yaw, 2.0 * M_PI)) < epsilon;
}

void appendDistinct(std::vector<Pose2D>& path, const Pose2D& pose) {
  if (path.empty() || !samePose(path.back(), pose)) {
    path.push_back(pose);
  }
}

}  // namespace

Planner::Planner(GridMap map)
    : dubinsLookup(
          Constants::headings * Constants::headings * Constants::dubinsWidth *
          Constants::dubinsWidth) {
  initializeLookups();
  setMap(std::move(map));
}

void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup.data());
  }
}

GridMap Planner::loadMapFromImage(
    const std::string& mapPath,
    int occupiedThreshold) {
  const cv::Mat image = cv::imread(mapPath, cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    throw std::runtime_error("Failed to read map image: " + mapPath);
  }

  GridMap map;
  map.resolution = Constants::cellSize;
  map.width = static_cast<unsigned int>(image.cols);
  map.height = static_cast<unsigned int>(image.rows);
  map.data.resize(static_cast<std::size_t>(map.width) * map.height);

  const int threshold = std::clamp(occupiedThreshold, 0, 255);
  for (int mapY = 0; mapY < image.rows; ++mapY) {
    const int imageY = image.rows - 1 - mapY;
    const auto* row = image.ptr<unsigned char>(imageY);
    for (int x = 0; x < image.cols; ++x) {
      map.data[static_cast<std::size_t>(mapY) * map.width + x] =
          row[x] < threshold ? 1 : 0;
    }
  }

  return map;
}

void Planner::setMap(GridMap map) {
  if (!map.isValid()) {
    throw std::invalid_argument("Hybrid A* requires a non-empty, valid grid map.");
  }

  grid = std::move(map);
  configurationSpace.updateGrid(&grid);

  const int width = static_cast<int>(grid.width);
  const int height = static_cast<int>(grid.height);
  bool** binaryMap = new bool*[width];
  for (int x = 0; x < width; ++x) {
    binaryMap[x] = new bool[height];
    for (int y = 0; y < height; ++y) {
      binaryMap[x][y] =
          grid.data[static_cast<std::size_t>(y) * grid.width + x] != 0;
    }
  }

  // DynamicVoronoi 接管 binaryMap 的所有权，并在析构时释放。
  voronoiDiagram.initializeMap(width, height, binaryMap);
  voronoiDiagram.update();
}

bool Planner::poseInMap(
    const Pose2D& pose,
    const char* label,
    std::string& errorMessage) const {
  if (!grid.isValid()) {
    errorMessage = "Cannot plan without a loaded map.";
    return false;
  }
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw)) {
    errorMessage = std::string(label) + " pose contains a non-finite value.";
    return false;
  }

  const float x = static_cast<float>(pose.x / grid.resolution);
  const float y = static_cast<float>(pose.y / grid.resolution);
  const float yaw = Helper::normalizeHeadingRad(static_cast<float>(pose.yaw));
  if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) {
    std::ostringstream message;
    message << label << " pose (x=" << pose.x << ", y=" << pose.y
            << ") is outside the " << grid.width << "x" << grid.height
            << " map.";
    errorMessage = message.str();
    return false;
  }

  if (!configurationSpace.configurationTest(x, y, yaw)) {
    std::ostringstream message;
    message << label << " pose (x=" << pose.x << ", y=" << pose.y
            << ", yaw=" << pose.yaw << ") is in collision.";
    errorMessage = message.str();
    return false;
  }

  return true;
}

PlanResult Planner::plan(const Pose2D& start, const Pose2D& goal) {
  const auto startedAt = std::chrono::steady_clock::now();
  PlanResult result;

  if (!poseInMap(start, "Start", result.message) ||
      !poseInMap(goal, "Goal", result.message)) {
    result.planningTimeMs = elapsedMilliseconds(startedAt);
    return result;
  }

  const int width = static_cast<int>(grid.width);
  const int height = static_cast<int>(grid.height);
  const int length = width * height * Constants::headings;
  std::vector<Node3D> nodes3D(length);
  std::vector<Node2D> nodes2D(width * height);

  const Node3D nGoal(
      static_cast<float>(goal.x / grid.resolution),
      static_cast<float>(goal.y / grid.resolution),
      Helper::normalizeHeadingRad(static_cast<float>(goal.yaw)),
      0,
      0,
      nullptr);
  Node3D nStart(
      static_cast<float>(start.x / grid.resolution),
      static_cast<float>(start.y / grid.resolution),
      Helper::normalizeHeadingRad(static_cast<float>(start.yaw)),
      0,
      0,
      nullptr);

  Node3D* solution = Algorithm::hybridAStar(
      nStart,
      nGoal,
      nodes3D.data(),
      nodes2D.data(),
      width,
      height,
      configurationSpace,
      dubinsLookup.data());
  if (solution == nullptr) {
    result.message = "No collision-free path was found.";
    result.planningTimeMs = elapsedMilliseconds(startedAt);
    return result;
  }
  if (!(*solution == nGoal)) {
    result.message = "Hybrid A* reached its search limit before the goal.";
    result.planningTimeMs = elapsedMilliseconds(startedAt);
    return result;
  }

  smoother.tracePath(solution);
  smoother.smoothPath(voronoiDiagram);

  appendDistinct(result.waypoints, start);
  for (const Pose2D& waypoint : toPath(smoother.getPath())) {
    appendDistinct(result.waypoints, waypoint);
  }
  appendDistinct(result.waypoints, goal);

  result.success = !result.waypoints.empty();
  result.message = result.success ? "Path planned successfully."
                                  : "Planner returned an empty path.";
  result.planningTimeMs = elapsedMilliseconds(startedAt);
  return result;
}

std::vector<Pose2D> Planner::toPath(const std::vector<Node3D>& nodePath) const {
  std::vector<Pose2D> path;
  path.reserve(nodePath.size());
  for (auto it = nodePath.rbegin(); it != nodePath.rend(); ++it) {
    path.push_back(Pose2D{
        it->getX() * grid.resolution,
        it->getY() * grid.resolution,
        Helper::normalizeHeadingRad(it->getT()),
    });
  }
  return path;
}
