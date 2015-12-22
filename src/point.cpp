#include <iostream>
#include <cmath>

#include "point.h"

using namespace std;

namespace gca {
  
  bool within_eps(point& l, point& r, double eps) {
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
  
}
