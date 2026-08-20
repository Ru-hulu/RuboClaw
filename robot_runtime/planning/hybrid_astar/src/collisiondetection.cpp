#include "collisiondetection.h"

#include <algorithm>

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) const {
  if (grid == nullptr || !grid->isValid()) {
    return false;
  }

  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = std::clamp(
      static_cast<int>(t / Constants::deltaHeadingRad),
      0,
      Constants::headings - 1);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && static_cast<unsigned int>(cX) < grid->width &&
        cY >= 0 && static_cast<unsigned int>(cY) < grid->height) {
      if (grid->data[cY * grid->width + cX]) {
        return false;
      }
    }
  }

  return true;
}
