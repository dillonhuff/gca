#ifndef GCA_TRIANGLE_H
#define GCA_TRIANGLE_H

#include <cmath>
#include <vector>

#include "geometry/point.h"
#include "geometry/polygon.h"
#include "geometry/matrix.h"

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

    inline point centroid() const
    { return (1.0 / 3.0) * (v1 + v2 + v3); }

    double area() const {
      double a = (v1  - v2).len();
      double b = (v2  - v3).len();
      double c = (v1  - v3).len();
      double s = (a + b + c) / 2.0;
      return sqrt(s*(s - a)*(s - b)*(s - c));
    }
  };

  bool in_projection(const triangle t, const point p);
  bool below(const triangle t, const point p);
  double z_at(const triangle t, double x, double y);
  bool intersects(const triangle t, const line l);

  double min_z(const vector<triangle>& triangles);
  bool is_upward_facing(const triangle& t, double tolerance);
  bool same_orientation(const triangle& x, const triangle& y, double tolerance);
  vector<oriented_polygon> mesh_bounds(const vector<triangle>& tris);
  ostream& operator<<(ostream& out, const triangle& t);

  void select_visible_triangles(vector<triangle>& triangles);

  vector<oriented_polygon> preprocess_triangles(vector<triangle>& triangles);
  bool intersects_triangles(line l, const vector<triangle>& triangles);

  triangle apply(const matrix<3, 3> m, const triangle& t);

  double distance_along(point normal, const triangle t);
}

#endif
