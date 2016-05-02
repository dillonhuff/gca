#include <cassert>

#include "geometry/matrix.h"

namespace gca {

  matrix<3, 3> rotate_onto(point a_v, point b_v) {
    point a = a_v.normalize();
    point b = b_v.normalize();
    point v = cross(a, b);
    double s = v.len();
    double c = a.dot(b);
    if (within_eps(s, 0)) {
      return identity<3, 3>();
    }
    double cs = ((1 - c) / (s*s));
    matrix<3, 3> id = identity<3, 3>();
    double v_coeffs[] = {0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0};
    matrix<3, 3> vx(v_coeffs);
    return id + vx + cs*(vx*vx); //matrix<3, 3>();
  }

  point operator*(const matrix<3, 3>& m, const point a) {
    double xr = m.get(0, 0) * a.x + m.get(0, 1) * a.y + m.get(0, 2) * a.z;
    double yr = m.get(1, 0) * a.x + m.get(1, 1) * a.y + m.get(1, 2) * a.z;
    double zr = m.get(2, 0) * a.x + m.get(2, 1) * a.y + m.get(2, 2) * a.z;
    point res(xr, yr, zr);
    return res;
  }
}
