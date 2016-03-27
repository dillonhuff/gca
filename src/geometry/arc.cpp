#include "geometry/arc.h"

namespace gca {

  point arc::value(double t) const {
    point x_axis = point(1, 0, start.z);
    cout << "x_axis = " << x_axis << endl;
    double s = angle_between(start - center, x_axis);
    cout << "s = " << s << endl;
    double e = angle_between(end - center, x_axis);
    cout << "e = " << e << endl;
    double theta_deg = (e - s)*t + s;
    cout << "Center = " << center << endl;
    double theta = (M_PI / 180)*theta_deg;
    cout << "theta = " << theta << endl;    
    cout << "sin(theta) = " << sin(theta) << endl;    
    return center + point(radius*cos(theta), radius*sin(theta), 0);
  }

}
