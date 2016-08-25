#ifndef GCA_RING_H
#define GCA_RING_H

#include "geometry/point.h"
#include "utils/check.h"

namespace gca {

  template<typename Ring>
  point ring_normal(const Ring& r) {
    double normal_x = 0.0;
    double normal_y = 0.0;
    double normal_z = 0.0;

    int i, j;
    for (i = 0, j = 1; i < r.size(); i++, j++) {
      if (j == r.size()) j = 0;

      point pi = r[i];
      point pj = r[j];
      
      normal_x += (((pi.z) + (pj.z)) *
		   ((pj.y) - (pi.y)));
      
      normal_y += (((pi.x) + (pj.x)) *
		   ((pj.z) - (pi.z)));
      
      normal_z += (((pi.y) + (pj.y)) *
		   ((pj.x) - (pi.x)));
    }
    // point l1 = r[1] - r[0];
    // point l2 = r[2] - r[0];
    // return cross(l1, l2).normalize();
    return point(normal_x, normal_y, normal_z).normalize();
  }

  template<typename Ring>
  void correct_winding_order(Ring& r, const point n) {
    double theta = angle_between(ring_normal(r), n);
    if (within_eps(theta, 0, 0.1)) { return; }

    std::cout << "Ring normal before = " << ring_normal(r) << std::endl;
      
    DBG_ASSERT(within_eps(theta, 180, 0.1));

    reverse(r);

    std::cout << "Ring normal after = " << ring_normal(r) << std::endl;
    std::cout << "Expected normal   = " << n << std::endl;

    double new_theta = angle_between(ring_normal(r), n);

    DBG_ASSERT(within_eps(new_theta, 0, 0.1));
  }
  
}

#endif
