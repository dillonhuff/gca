#ifndef GCA_PLANE_H
#define GCA_PLANE_H

#include "geometry/point.h"

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
  };
}

#endif
