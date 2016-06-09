#include "geometry/point.h"

namespace gca {

  struct workpiece {
    point sides[3];

    workpiece(double x, double y, double z) {
      sides[0] = point(x, 0, 0);
      sides[1] = point(0, y, 0);
      sides[2] = point(0, 0, z);
    }

    workpiece(point x, point y, point z) {
      sides[0] = x;
      sides[1] = y;
      sides[2] = z;
    }

  };

}
