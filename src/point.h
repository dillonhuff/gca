#ifndef GCA_POINT_H
#define GCA_POINT_H

#include "point.h"

namespace gca {

  class point {
  public:
    double x, y, z;

  point(double xp, double yp, double zp) :
    x(xp), y(yp), z(zp) {}

    bool operator==(const point& other) const {
      return x == other.x && y == other.y && z == other.z;
    }

    point operator+(const point& other) const {
      return point(x + other.x, y + other.y, z + other.z);
    }

  };

  bool within_eps(point& l, point& r, double eps=0.0000001);
  bool within_eps(double l, double r, double eps=0.0000001);
  
}

#endif
