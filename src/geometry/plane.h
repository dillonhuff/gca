#ifndef GCA_PLANE_H
#define GCA_PLANE_H

#include <boost/optional.hpp>

#include "geometry/line.h"
#include "utils/algorithm.h"

namespace gca {

  class plane {
  protected:
    point norm;
    point p;

  public:
    plane(const point p_n, const point p_p)
      : norm(p_n.normalize()), p(p_p) {}

    inline point normal() const { return norm; }
    inline point pt() const { return p; }

    inline plane flip() const { return plane(-1*normal(), pt()); }
    inline plane slide(const double d) const
    { return plane(normal(), pt() + d*normal()); }
  };

  boost::optional<point>
  plane_intersection(const plane p, const line l);

  double signed_distance(const plane p, const point v);
  
  template<>
  class intersection_impl<const plane, const line> {
  public:
    typedef boost::optional<point> result_type;

    static
    result_type apply(const plane p, const line l) {
      return plane_intersection(p, l);
    }
  };


  template<>
  class intersection_impl<const line, const plane> {
  public:
    typedef boost::optional<point> result_type;

    static
    result_type apply(const line l, const plane p) {
      return plane_intersection(p, l);
    }
  };

  std::vector<point> project(const plane pl, const std::vector<point>& pts);
  point project(const plane pl, const point p);

  
}

#endif
