#include "geometry/plane.h"

namespace gca {

  double signed_distance(const plane p, const point v) {
    point u = v - p.pt();
    return signed_distance_along(u, p.normal());
  }

  boost::optional<point>
  plane_intersection(const plane p, const line l) {
    double dl = signed_distance(p, l.start);
    double dr = signed_distance(p, l.end);
    if (dl*dr > 0) {
      return boost::none;
    }
    point u = (l.start - p.pt()).normalize();
    return l.start - dl*u;
  }

}
