#ifndef GCA_RING_H
#define GCA_RING_H

#include "geometry/point.h"
#include "utils/check.h"

namespace gca {

  template<typename Ring>
  point ring_normal(const Ring& r) {
    point l1 = r[1] - r[0];
    point l2 = r[2] - r[0];
    return cross(l1, l2).normalize();
  }

  template<typename Ring>
  void correct_winding_order(Ring& r, const point n) {
    double theta = angle_between(ring_normal(r), n);
    if (within_eps(theta, 0, 0.1)) { return; }

    std::cout << "Ring normal before = " << ring_normal(r) << std::endl;
      
    DBG_ASSERT(within_eps(theta, 180, 0.1));

    reverse(r);

    std::cout << "Ring normal after = " << ring_normal(r) << std::endl;

    double new_theta = angle_between(ring_normal(r), n);

    DBG_ASSERT(within_eps(new_theta, 0, 0.1));
  }
  
}

#endif
