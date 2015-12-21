#include <iostream>
#include <math.h>

#include "point.h"

using namespace std;

namespace gca {
  
  bool within_eps(point& l, point& r, double eps) {
    double xd = l.x - r.x;
    double yd = l.y - r.y;
    double zd = l.z - r.z;
    double diff = sqrt(xd*xd + yd*yd + zd*zd);
    cout << "diff = " << diff << endl;
    cout << "eps = " << eps << endl;
    return diff <= eps;
  }
  
}
