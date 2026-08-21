#ifndef HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_H

#include <string>
#include <vector>

#include "algorithm.h"
#include "collisiondetection.h"
#include "constants.h"
#include "dynamicvoronoi.h"
#include "planning_types.h"
#include "smoother.h"

namespace HybridAStar {

// 封装地图加载、碰撞空间构建、Hybrid A* 搜索和路径平滑，不依赖 ROS。
class Planner {
 public:
  explicit Planner(GridMap map);
  Planner(const Planner&) = delete;
  Planner& operator=(const Planner&) = delete;

  // 将灰度图片转换为栅格地图：暗像素为障碍，图片底边对应地图 y=0。
  static GridMap loadMapFromImage(
      const std::string& mapPath,
      int occupiedThreshold = 128);

  // 根据起终点二维位姿计算路径；坐标使用地图坐标系，单位为米和弧度。
  PlanResult plan(const Pose2D& start, const Pose2D& goal);

  const GridMap& map() const { return grid; }

 private:
  void initializeLookups();
  void setMap(GridMap map);
  bool poseInMap(
      const Pose2D& pose,
      const char* label,
      std::string& errorMessage) const;
  std::vector<Node3D> densifyPrimitivePath(
      const std::vector<Node3D>& tracedPath) const;
  std::vector<Pose2D> toPath(const std::vector<Node3D>& nodePath) const;

  Smoother smoother;
  CollisionDetection configurationSpace;
  DynamicVoronoi voronoiDiagram;
  GridMap grid;
  std::vector<float> dubinsLookup;
};

}  // namespace HybridAStar

#endif  // HYBRID_ASTAR_PLANNER_H
