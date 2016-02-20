#include <iostream>
#include <cmath>

#include "point.h"

using namespace std;

namespace gca {
  
  bool within_eps(const point& l, const point& r, double eps) {
    double xd = l.x - r.x;
    double yd = l.y - r.y;
    double zd = l.z - r.z;
    double diff = sqrt(xd*xd + yd*yd + zd*zd);
    return diff <= eps;
  }

  bool within_eps(double l, double r, double eps) {
    double diff = abs(l - r);
    return diff <= eps;
  }

  point point::rotate_z(double degrees) const {
    double theta_rad = (M_PI/180)*degrees;
    double new_x = cos(theta_rad)*x - sin(theta_rad)*y;
    double new_y = sin(theta_rad)*x + cos(theta_rad)*y;
    return point(new_x, new_y, z);
  }

  void point::print(ostream& s) const {
    s << "(" << x << ", " << y << ", " << z << ")";
  }

  double point::len() const {
    return sqrt(x*x + y*y + z*z);
  }

  point operator*(double a, const point& p) {
    return point(a*p.x, a*p.y, a*p.z);
  }

  double angle_between(point u, point v) {
    double rads = acos((u.dot(v)) / (u.len() * v.len()));
    return (180.0/M_PI)*rads;
  }
  
  ostream& operator<<(ostream& s, const point& p) {
    p.print(s);
    return s;
  }

  point extend_back(point start, point end, double l) {
    point se = end - start;
    point sp = start - ((l/se.len())*se);
    return sp;
  }
}
