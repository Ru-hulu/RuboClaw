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

constexpr double kPrimitiveSampleSpacingMeters = 0.3;

double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

bool isReversePrimitive(int primitive) {
  return primitive >= Node3D::dir;
}

MotionDirection directionFromPrimitive(int primitive) {
  return isReversePrimitive(primitive) ? MotionDirection::Reverse
                                       : MotionDirection::Forward;
}

bool validPrimitive(int primitive) {
  const int primitiveIndex =
      isReversePrimitive(primitive) ? primitive - Node3D::dir : primitive;
  return primitiveIndex >= 0 && primitiveIndex < Node3D::dir;
}

double primitiveLengthGrid(int primitive) {
  if (!validPrimitive(primitive)) {
    return 0.0;
  }
  return static_cast<double>(Node3D::dx[0]);
}

Node3D samplePrimitive(
    const Node3D& parent,
    int primitive,
    double distanceGrid) {
  const bool reverse = isReversePrimitive(primitive);
  const int primitiveIndex = reverse ? primitive - Node3D::dir : primitive;
  const double directionSign = reverse ? -1.0 : 1.0;
  const double parentYaw = parent.getT();

  double localX = 0.0;
  double localY = 0.0;
  double yawDelta = 0.0;
  const double primitiveLength = primitiveLengthGrid(primitive);
  const double ratio = primitiveLength > 0.0
                           ? std::clamp(
                                 distanceGrid / primitiveLength,
                                 0.0,
                                 1.0)
                           : 0.0;
  const double primitiveYawDelta =
      static_cast<double>(Node3D::dt[primitiveIndex]);

  if (std::abs(primitiveYawDelta) < 1e-9) {
    localX = directionSign * distanceGrid;
  } else {
    yawDelta = (reverse ? -primitiveYawDelta : primitiveYawDelta) * ratio;
    const double radius = primitiveLength / std::abs(primitiveYawDelta);
    const double absYawDelta = std::abs(yawDelta);
    const double yawSign = yawDelta >= 0.0 ? 1.0 : -1.0;
    localX = directionSign * radius * std::sin(absYawDelta);
    localY =
        -directionSign * yawSign * radius * (1.0 - std::cos(absYawDelta));
  }

  const double cosYaw = std::cos(parentYaw);
  const double sinYaw = std::sin(parentYaw);
  return Node3D(
      static_cast<float>(parent.getX() + localX * cosYaw - localY * sinYaw),
      static_cast<float>(parent.getY() + localX * sinYaw + localY * cosYaw),
      Helper::normalizeHeadingRad(static_cast<float>(parentYaw + yawDelta)),
      0,
      0,
      nullptr,
      primitive);
}

double poseDistanceGrid(const Node3D& left, const Node3D& right) {
  return std::hypot(
      static_cast<double>(right.getX() - left.getX()),
      static_cast<double>(right.getY() - left.getY()));
}

bool sameGridPose(
    const Node3D& node,
    const Pose2D& pose,
    double resolution) {
  constexpr double positionTolerance = 1e-6;
  constexpr double yawTolerance = 1e-6;
  const double x = pose.x / resolution;
  const double y = pose.y / resolution;
  return std::hypot(node.getX() - x, node.getY() - y) < positionTolerance &&
         std::abs(std::remainder(node.getT() - pose.yaw, 2.0 * M_PI)) <
             yawTolerance;
}

bool matchesPrimitiveEndpoint(
    const Node3D& parent,
    const Node3D& child,
    int primitive) {
  if (!validPrimitive(primitive)) {
    return false;
  }

  const Node3D expected =
      samplePrimitive(parent, primitive, primitiveLengthGrid(primitive));
  constexpr double positionTolerance = 1e-3;
  constexpr double yawTolerance = 1e-3;
  return poseDistanceGrid(expected, child) < positionTolerance &&
         std::abs(std::remainder(expected.getT() - child.getT(), 2.0 * M_PI)) <
             yawTolerance;
}

Node3D interpolatePoseSegment(
    const Node3D& parent,
    const Node3D& child,
    int primitive,
    double ratio) {
  const double yaw =
      static_cast<double>(parent.getT()) +
      std::remainder(
          static_cast<double>(child.getT() - parent.getT()),
          2.0 * M_PI) *
          ratio;
  return Node3D(
      static_cast<float>(
          parent.getX() + (child.getX() - parent.getX()) * ratio),
      static_cast<float>(
          parent.getY() + (child.getY() - parent.getY()) * ratio),
      Helper::normalizeHeadingRad(static_cast<float>(yaw)),
      0,
      0,
      nullptr,
      primitive);
}

void normalizeEndpointDirections(std::vector<Pose2D>& path) {
  if (path.size() < 2) {
    return;
  }
  path.front().direction = path[1].direction;
  path.back().direction = path[path.size() - 2].direction;
}

void pinEndpointPositions(
    std::vector<Pose2D>& path,
    const Pose2D& start,
    const Pose2D& goal) {
  if (path.empty()) {
    return;
  }
  path.front().x = start.x;
  path.front().y = start.y;
  path.back().x = goal.x;
  path.back().y = goal.y;
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
  std::vector<Node3D> tracedPath = smoother.getPath();
  if (!tracedPath.empty() &&
      !sameGridPose(tracedPath.front(), goal, grid.resolution)) {
    tracedPath.insert(
        tracedPath.begin(),
        Node3D(
            static_cast<float>(goal.x / grid.resolution),
            static_cast<float>(goal.y / grid.resolution),
            Helper::normalizeHeadingRad(static_cast<float>(goal.yaw)),
            0,
            0,
            nullptr,
            tracedPath.front().getPrim()));
  }
  if (!tracedPath.empty() &&
      !sameGridPose(tracedPath.back(), start, grid.resolution)) {
    tracedPath.push_back(Node3D(
        static_cast<float>(start.x / grid.resolution),
        static_cast<float>(start.y / grid.resolution),
        Helper::normalizeHeadingRad(static_cast<float>(start.yaw)),
        0,
        0,
        nullptr,
        tracedPath.back().getPrim()));
  }
  smoother.setPath(densifyPrimitivePath(tracedPath));
  smoother.smoothPath(voronoiDiagram);

  result.waypoints = toPath(smoother.getPath());
  pinEndpointPositions(result.waypoints, start, goal);
  normalizeEndpointDirections(result.waypoints);

  result.success = !result.waypoints.empty();
  result.message = result.success ? "Path planned successfully."
                                  : "Planner returned an empty path.";
  result.planningTimeMs = elapsedMilliseconds(startedAt);
  return result;
}

std::vector<Node3D> Planner::densifyPrimitivePath(
    const std::vector<Node3D>& tracedPath) const {
  if (tracedPath.size() <= 1) {
    return tracedPath;
  }

  std::vector<Node3D> startToGoal(tracedPath.rbegin(), tracedPath.rend());
  std::vector<Node3D> denseStartToGoal;
  denseStartToGoal.reserve(startToGoal.size() * 3);
  denseStartToGoal.push_back(startToGoal.front());

  const double sampleSpacingGrid =
      kPrimitiveSampleSpacingMeters / grid.resolution;
  for (std::size_t index = 1; index < startToGoal.size(); ++index) {
    const Node3D& parent = startToGoal[index - 1];
    const Node3D& child = startToGoal[index];
    const int primitive = child.getPrim();
    const bool usePrimitiveArc =
        validPrimitive(primitive) &&
        matchesPrimitiveEndpoint(parent, child, primitive);
    const double segmentLength =
        usePrimitiveArc ? primitiveLengthGrid(primitive)
                        : poseDistanceGrid(parent, child);

    if (segmentLength > sampleSpacingGrid) {
      for (double distance = sampleSpacingGrid; distance < segmentLength;
           distance += sampleSpacingGrid) {
        if (usePrimitiveArc) {
          denseStartToGoal.push_back(
              samplePrimitive(parent, primitive, distance));
        } else {
          denseStartToGoal.push_back(interpolatePoseSegment(
              parent,
              child,
              primitive,
              distance / segmentLength));
        }
      }
    }
    denseStartToGoal.push_back(child);
  }

  return std::vector<Node3D>(
      denseStartToGoal.rbegin(),
      denseStartToGoal.rend());
}

std::vector<Pose2D> Planner::toPath(const std::vector<Node3D>& nodePath) const {
  std::vector<Pose2D> path;
  path.reserve(nodePath.size());
  for (auto it = nodePath.rbegin(); it != nodePath.rend(); ++it) {
    path.push_back(Pose2D{
        it->getX() * grid.resolution,
        it->getY() * grid.resolution,
        Helper::normalizeHeadingRad(it->getT()),
        directionFromPrimitive(it->getPrim()),
    });
  }
  return path;
}
