#ifndef GCA_POLYGON_H
#define GCA_POLYGON_H

#include "geometry/point.h"

namespace gca {

  class oriented_polygon {
  public:
    point normal;
    vector<point> vertices;

    oriented_polygon() :
      normal(point(0, 0, 0)), vertices({}) {}

    oriented_polygon(point normalp, const vector<point>& verticesp) :
      normal(normalp), vertices(verticesp) {}
  };

  bool is_horizontal(const oriented_polygon& p);
}

#endif
