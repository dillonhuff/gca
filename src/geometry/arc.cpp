#include "geometry/arc.h"

namespace gca {

  point arc::value(double t) const {
    point sd = start - center;
    point ed = end - center;
    cout << "center = " << center << endl;
    cout << "sd = " << sd << endl;
    cout << "ed = " << ed << endl;
    double theta = angle_between(sd, ed);
    cout << "theta = " << theta << endl;
    double tv = dir == COUNTERCLOCKWISE ? t : (-1)*t;
    double psi = tv*theta;
    cout << "psi = " << psi << endl;
    return center + sd.rotate_z(psi);
  }

}
