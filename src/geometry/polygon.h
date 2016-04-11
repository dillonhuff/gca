#ifndef GCA_POLYGON_H
#define GCA_POLYGON_H

#include "geometry/point.h"

namespace gca {

  class oriented_polygon {
  public:
    point normal;
    vector<point> vertices;

    oriented_polygon(point normalp, const vector<point>& verticesp) :
      normal(normalp), vertices(verticesp) {}
  };
}

#endif
