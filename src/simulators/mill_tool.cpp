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

  bool cylindrical_bit::contains(const point tool_location,
				 const point other_pt) {//, double resolution, int i, int j) const {
    // double bl_corner_x = origin.x + i*resolution;
    // double bl_corner_y = origin.y + j*resolution;
    if (!in_circle(tool_location, other_pt.x, other_pt.y)) {
      return false;
    }

    return tool_location.z < other_pt.z;
  }
  
  bool cylindrical_bit::contains(point p, const point origin, double resolution, int i, int j) const {
    double bl_corner_x = origin.x + i*resolution;
    double bl_corner_y = origin.y + j*resolution;
    return in_circle(p, bl_corner_x, bl_corner_y);
  }

  bool ball_nosed::in_circle(point p, double x, double y) const {
    double xd = x - p.x;
    double yd = y - p.y;
    double r = diameter / 2.0;
    double val = sqrt(xd*xd + yd*yd);
    return val <= r;
  }

  bool ball_nosed::contains(point p, const point origin, double resolution, int i, int j) const {
    double bl_corner_x = origin.x + i*resolution;
    double bl_corner_y = origin.y + j*resolution;
    if (!in_circle(p, bl_corner_x, bl_corner_y)) {
      return false;
    }

    point sphere_center = p + point(0, 0, radius);
    double zsq = radius*radius - sphere_center.x*sphere_center.x -
      sphere_center.y*sphere_center.y;
    double z = sqrt(zsq);

    return z < 
  }
  
}
