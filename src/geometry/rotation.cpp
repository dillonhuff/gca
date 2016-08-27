#include <boost/numeric/ublas/io.hpp>

#include "geometry/rotation.h"
#include "utils/check.h"

namespace gca {

  rotation rotate_from_to(const point from, const point to) {
    double theta = angle_between(from, to);

    std::cout << "theta = " << theta << endl;
    
    if (within_eps(theta, 0, 0.01)) {
      return boost::numeric::ublas::identity_matrix<double>(3);
    }

    if (within_eps(theta, 180, 0.01)) {
      return -1*boost::numeric::ublas::identity_matrix<double>(3);
    }
    
    point from_unit = from.normalize();
    point to_unit = to.normalize();

    // point v = cross(from_unit, to_unit).normalize();
    // //    double s = v.len();
    // double dt = from_unit.dot(to_unit);

    // double th = safe_acos(dt / (from_unit.len() * to_unit.len()));

    point v = cross(from, to).normalize();
    theta = safe_acos(from.dot(to) / (from.len() * to.len()));

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

    const boost::numeric::ublas::matrix<double> vxc = vx;
    
    auto vx2 = prod(vx, vx);
    const ublas::matrix<double> r =
      id + sin(theta) * vxc + (1.0 - cos(theta))*vx2; //id + vxc + ((1.0 - c)/(s*s))*vx2;

    cout << "r * from_unit = " << times_3(r, from_unit) << endl;

    double d = determinant(r);
    if (!(within_eps(d, 1.0, 0.001))) {
      cout << "ERROR: determinant of rotation = " << d << endl;
      cout << "from = " << from << endl;
      cout << "to = " << to << endl;
      cout << "from unit normal = " << from_unit << endl;
      cout << "to unit normal = " << to_unit << endl;

      cout << "theta = " << theta << endl;
      cout << r << endl;

      point rfu = times_3(r, from_unit);
      double res_angle = angle_between(rfu, to_unit);

      cout << "r * from_unit = " << rfu << endl;
      cout << "resulting angle = " << res_angle << endl;

      // cout << "c = " << c << endl;
      // cout << "s = " << s << endl;
      // cout << "v = " << v << endl;

      cout << "vx = " << endl;
      cout << vx << endl;
      
      DBG_ASSERT(false);
    }

    if (!(within_eps(angle_between(times_3(r, from_unit), to_unit), 0.0, 0.1))) {
      cout << "ERROR: Incorrect rotation " << endl;
      cout << r << endl;

      cout << "from = " << from << endl;
      cout << "to = " << to << endl;
      cout << "from unit normal = " << from_unit << endl;
      cout << "to unit normal = " << to_unit << endl;
      
      cout << "r*" << from_unit << " = " << times_3(r, from_unit) << " != " << to_unit << endl;
      // cout << "c = " << c << endl;
      // cout << "s = " << s << endl;
      // cout << "v = " << v << endl;
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
