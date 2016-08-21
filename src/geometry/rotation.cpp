#include <boost/numeric/ublas/io.hpp>

#include "geometry/rotation.h"
#include "utils/check.h"

namespace gca {

  rotation rotate_from_to(const point from, const point to) {
    if (within_eps(angle_between(from, to), 0, 0.01)) {
      return boost::numeric::ublas::identity_matrix<double>(3);
    }

    point from_unit = from.normalize();
    point to_unit = to.normalize();

    point v = cross(from_unit, to_unit);
    double s = v.len();
    double c = from_unit.dot(to_unit);    
    
    boost::numeric::ublas::matrix<double> vx(3, 3);
    vx(0, 0) = 0;
    vx(0, 1) = -v.z;
    vx(0, 2) = -v.y;

    vx(1, 0) = v.z;
    vx(1, 1) = 0;
    vx(1, 2) = -v.x;

    vx(2, 0) = -v.y;
    vx(2, 1) = v.x;
    vx(2, 2) = 0;

    auto id = boost::numeric::ublas::identity_matrix<double>(3);

    auto vx2 = prod(vx, vx);
    const ublas::matrix<double> r = id + vx + ((1.0 - c)/(s*s))*vx2;

    if (!(within_eps(determinant(r), 1.0, 0.001))) {
      cout << "ERROR: determinant of rotation = " << determinant(r) << endl;
      cout << r << endl;

      cout << "c = " << c << endl;
      cout << "s = " << s << endl;
      cout << "v = " << v << endl;
      DBG_ASSERT(false);
    }

    if (!(within_eps(angle_between(times_3(r, from), to), 0.0, 0.1))) {
      cout << "ERROR: Incorrect rotation " << endl;
      cout << r << endl;

      cout << "r*" << from << " = " << times_3(r, from) << " != " << to << endl;
      cout << "c = " << c << endl;
      cout << "s = " << s << endl;
      cout << "v = " << v << endl;
      DBG_ASSERT(false);
    }
    
    return r;
  }

  triangular_mesh apply(const rotation& r, const triangular_mesh& m) {
    triangular_mesh rotated =
      m.apply([r](const point p)
	      { return times_3(r, p); });
    return rotated;
  }

  std::vector<point> apply(const rotation& r, const std::vector<point>& pts) {
    std::vector<point> rpts;
    for (auto p : pts) {
      rpts.push_back(times_3(r, p));
    }
    return rpts;
  }
  
}
