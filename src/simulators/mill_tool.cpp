#include <cmath>

#include "simulators/mill_tool.h"

namespace gca {

  bool cylindrical_bit::in_circle(point p, double x, double y) const {
    double xd = x - p.x;
    double yd = y - p.y;
    double r = diameter / 2.0;
    double val = sqrt(xd*xd + yd*yd);
    return val <= r;
  }

  bool cylindrical_bit::contains(point p, double resolution, int i, int j) const {
    double bl_corner_x = i*resolution;
    double bl_corner_y = j*resolution;
    return in_circle(p, bl_corner_x, bl_corner_y);
  }

}
