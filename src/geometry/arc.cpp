#include "geometry/arc.h"

namespace gca {

  point arc::value(double t) const {
    point x_axis = point(1, 0, start.z);
    double s = angle_between(start - center, x_axis);
    double e = angle_between(end - center, x_axis);
    double theta_deg = (e - s)*t + s;
    double theta = (M_PI / 180)*theta_deg;
    return center + point(radius*cos(theta), radius*sin(theta), 0);
  }

}
