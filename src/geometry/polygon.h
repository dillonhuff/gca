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

    inline point pt(unsigned i) const { return vertices[i]; }
  };

  bool contains(const oriented_polygon& g, point p);
  bool contains(const oriented_polygon& poly,
		const oriented_polygon& maybe_contained);

  bool is_horizontal(const oriented_polygon& p);
}

#endif
