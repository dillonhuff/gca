#include <cassert>
#include <cmath>
#include <iostream>

#include "point.h"

using namespace std;

namespace gca {

  double safe_acos(double v) {
    if (within_eps(v, -1)) { return M_PI; }
    if (within_eps(v, 1)) { return 0.0; }
    assert(-1 <= v && v <= 1);
    return acos(v);
  }
  
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

  point point::normalize() const {
    double l = len();
    assert(!within_eps(l, 0.0));
    return point(x / l, y / l, z / l);
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
    if (within_eps(u, v)) { return 0.0; }
    double l = u.len() * v.len();
    double d = u.dot(v);
    if (within_eps(d, 0)) { return 180.0; }
    double m = (u.dot(v)) / l;
    double rads = safe_acos(m);
    return (180.0/M_PI)*rads;
  }
  
  ostream& operator<<(ostream& s, const point& p) {
    p.print(s);
    return s;
  }

  ostream& operator<<(ostream& s, const vector<point>& p) {
    for (auto pt : p) { pt.print(s); s << " "; }
    return s;
  }
  
  point extend_back(point start, point end, double l) {
    point se = end - start;
    point sp = start - ((l/se.len())*se);
    return sp;
  }
}
