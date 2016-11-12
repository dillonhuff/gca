#ifndef GCA_ROTATION_H
#define GCA_ROTATION_H

#include "geometry/matrix.h"
#include "geometry/polygon_3.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  typedef ublas::matrix<double> rotation;

  rotation rotate_from_to(const point from, const point to);

  triangular_mesh apply(const rotation& r, const triangular_mesh& m);

  std::vector<point> apply(const rotation& r, const std::vector<point>& pts);

  triangle apply(const rotation& r, const triangle tri);

  polyline apply(const rotation& r, const polyline& p);

  polygon_3 apply(const rotation& r, const polygon_3& p);

  boost_multipoly_2
  to_boost_multipoly_2(const rotation& r, const std::vector<polygon_3>& lines);

  polygon_3 apply_no_check(const rotation& r, const polygon_3& p);
}

#endif
