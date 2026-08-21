#include "smoother.h"

using namespace HybridAStar;
//###################################################
//                                     CUSP DETECTION
//###################################################
inline bool isCusp(const std::vector<Node3D>& path, int i) {
  bool revim2 = path[i - 2].getPrim() >= Node3D::dir;
  bool revim1 = path[i - 1].getPrim() >= Node3D::dir;
  bool revi   = path[i].getPrim() >= Node3D::dir;
  bool revip1 = path[i + 1].getPrim() >= Node3D::dir;
  bool revip2 = path[i + 2].getPrim() >= Node3D::dir;

  return (
      revim2 != revim1 ||
      revim1 != revi ||
      revi != revip1 ||
      revip1 != revip2);
}
//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath(DynamicVoronoi& voronoi) {
  // load the current voronoi diagram into the smoother
  this->voronoi = &voronoi;
  this->width = voronoi.getSizeX();
  this->height = voronoi.getSizeY();
  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // the maximum iterations for the gd smoother
  int maxIterations = 500;
  // the lenght of the path in number of nodes
  int pathLength = 0;

  // path objects with all nodes oldPath the original, newPath the resulting smoothed path
  pathLength = path.size();
  std::vector<Node3D> newPath = path;

  // descent along the gradient untill the maximum number of iterations has been reached
  float totalWeight = wSmoothness + wObstacle;

  while (iterations < maxIterations) {

    // choose the first three nodes of the path
    for (int i = 2; i < pathLength - 2; ++i) {

      Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
      Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
      Vector2D xi(newPath[i].getX(), newPath[i].getY());
      Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
      Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
      Vector2D correction;


      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      if (isCusp(newPath, i)) { continue; }

      correction = correction - obstacleTerm(xi);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid
      correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid

      xi = xi + alpha * correction/totalWeight;
      newPath[i].setX(xi.getX());
      newPath[i].setY(xi.getY());
      Vector2D Dxi = xi - xim1;
      newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));

    }

    iterations++;
  }

  path = newPath;
}

void Smoother::tracePath(const Node3D* node, int i, std::vector<Node3D> path) {
  if (node == nullptr) {
    this->path = path;
    return;
  }

  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}

//###################################################
//                                      OBSTACLE TERM
//###################################################
Vector2D Smoother::obstacleTerm(Vector2D xi) {
  Vector2D gradient;
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi->getDistance(xi.getX(), xi.getY());
  // the vector determining where the obstacle is
  int x = (int)xi.getX();
  int y = (int)xi.getY();
  // if the node is within the map
  if (x < width && x >= 0 && y < height && y >= 0) {
    Vector2D obsVct(xi.getX() - voronoi->data[(int)xi.getX()][(int)xi.getY()].obstX,
                    xi.getY() - voronoi->data[(int)xi.getX()][(int)xi.getY()].obstY);

    // the closest obstacle is closer than desired correct the path for that
    if (obsDst < obsDMax) {
      return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
    }
  }
  return gradient;
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
  return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}
