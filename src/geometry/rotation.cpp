#include <boost/numeric/ublas/io.hpp>

#include "geometry/rotation.h"
#include "utils/check.h"

namespace gca {

  void test_rotation(const point from_unit, const point to_unit, const rotation& r) {
    double d = determinant(r);
    if (!(within_eps(d, 1.0, 0.001))) {
      cout << "ERROR: determinant of rotation = " << d << endl;
      cout << "from unit normal = " << from_unit << endl;
      cout << "to unit normal = " << to_unit << endl;

      double theta = angle_between(from_unit, to_unit);
      cout << "theta = " << theta << endl;
      cout << r << endl;

      point rfu = times_3(r, from_unit);
      double res_angle = angle_between(rfu, to_unit);

      cout << "r * from = " << rfu << endl;
      cout << "resulting angle = " << res_angle << endl;

      // cout << "c = " << c << endl;
      // cout << "s = " << s << endl;
      // cout << "v = " << v << endl;

      // cout << "vx = " << endl;
      // cout << vx << endl;
      
      DBG_ASSERT(false);
    }

    if (!(within_eps(angle_between(times_3(r, from_unit), to_unit), 0.0, 0.1))) {
      cout << "ERROR: Incorrect rotation " << endl;
      cout << r << endl;

      cout << "from unit normal = " << from_unit << endl;
      cout << "to unit normal = " << to_unit << endl;
      
      cout << "r*" << from_unit << " = " << times_3(r, from_unit) << " != " << to_unit << endl;

      DBG_ASSERT(false);
    }
    
  }  

  rotation rotate_unit_from_to_regular(const point from_unit, const point to_unit) {
    point v = cross(from_unit, to_unit);
    double s = v.len();
    double c = dot(from_unit, to_unit);

    boost::numeric::ublas::matrix<double> vx(3, 3);
    vx(0, 0) = 0;
    vx(0, 1) = -v.z;
    vx(0, 2) = v.y;

    vx(1, 0) = v.z;
    vx(1, 1) = 0;
    vx(1, 2) = -v.x;

    vx(2, 0) = -v.y;
    vx(2, 1) = v.x;
    vx(2, 2) = 0;

    const boost::numeric::ublas::matrix<double> id =
      boost::numeric::ublas::identity_matrix<double>(3);

    const boost::numeric::ublas::matrix<double> vxc = vx;
    
    const boost::numeric::ublas::matrix<double> vx2 = prod(vx, vx);

    const ublas::matrix<double> r =
      id + vxc + ((1.0 - c)/(s*s))*vx2;

    return r;
  }

  rotation rotate_unit_from_to(const point from_unit, const point to_unit) {

    DBG_ASSERT(within_eps(from_unit.len(), 1.0, 0.0000001));
    DBG_ASSERT(within_eps(to_unit.len(),   1.0, 0.0000001));
    
    double theta = angle_between(from_unit, to_unit);

    if (within_eps(theta, 0, 0.01)) {
      return boost::numeric::ublas::identity_matrix<double>(3);
    }

    if (within_eps(theta, 180, 0.01)) {
      return -1*boost::numeric::ublas::identity_matrix<double>(3);
    }

    const ublas::matrix<double> r =
      rotate_unit_from_to_regular(from_unit, to_unit);

    test_rotation(from_unit, to_unit, r);

    return r;
    
  }

  rotation rotate_from_to(const point from, const point to) {
    point from_unit = from.normalize();
    point to_unit = to.normalize();

    return rotate_unit_from_to(from_unit, to_unit);
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
