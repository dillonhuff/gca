#ifndef GCA_GEOMETRY_ARC_H
#define GCA_GEOMETRY_ARC_H

#include <cassert>
#include <cmath>

#include "geometry/direction.h"
#include "geometry/point.h"

namespace gca {

  struct arc {
    point start, end, center;
    double radius;
    direction dir;

    arc(point startp, point endp, point start_offsetp, direction dirp) :
      start(startp), end(endp), center(start + start_offsetp),
      radius((start - center).len()), dir(dirp) {
      assert(within_eps(start.z, end.z));
    }

    inline point value(double t) const {
      point x_axis = point(1, 0, start.z);
      double s = angle_between(start.normalize(), x_axis);
      double e = angle_between(end.normalize(), x_axis);
      cout << "s = " << s << endl;
      cout << "e = " << e << endl;
      double theta = (e - s)*t + s;
      return center + point(radius*sin(theta), radius*cos(theta), 0);
    }
  };
}

#endif
