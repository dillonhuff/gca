#ifndef GCA_TRIANGLE_H
#define GCA_TRIANGLE_H

#include <vector>

#include "geometry/point.h"
#include "geometry/polygon.h"

using namespace std;

namespace gca {

  struct triangle {
    point normal;
    point v1;
    point v2;
    point v3;

    triangle(point normalp, point v1p, point v2p, point v3p) :
      normal(normalp), v1(v1p), v2(v2p), v3(v3p) {}

    vector<line> edges() const {
      return {line(v1, v2), line(v2, v3), line(v3, v1)};
    }
  };

  bool in_projection(const triangle t, const point p);
  bool below(const triangle t, const point p);

  double min_z(const vector<triangle>& triangles);
  bool is_upward_facing(const triangle& t, double tolerance);
  bool same_orientation(const triangle& x, const triangle& y, double tolerance);
  vector<oriented_polygon> mesh_bounds(const vector<triangle>& tris);
  ostream& operator<<(ostream& out, const triangle& t);

  vector<vector<triangle>> millable_surfaces(const vector<triangle>& tris);
}

#endif
