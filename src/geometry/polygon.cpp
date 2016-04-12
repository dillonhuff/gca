#include "geometry/polygon.h"

namespace gca {
  
  bool is_horizontal(const oriented_polygon& p) {
    return within_eps(p.normal.z, 1.0, 0.001);
  }
  
}
