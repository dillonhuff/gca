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

    point value(double t) const;
  };
}

#endif
