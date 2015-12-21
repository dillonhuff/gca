#ifndef GCA_POINT_H
#define GCA_POINT_H

#include "point.h"

namespace gca {

  class point {
  public:
    double x, y, z;

  point(double xp, double yp, double zp) :
    x(xp), y(yp), z(zp) {}

  };

  bool within_eps(point& l, point& r, double eps=0.0000001);
  
}

#endif
