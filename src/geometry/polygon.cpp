#include <cmath>

#include "geometry/polygon.h"

namespace gca {

  /*
    Return the angle between two vectors on a plane
    The angle is from vector 1 to vector 2, positive anticlockwise
    The result is between -pi -> pi
  */
  double angle_2d(double x1, double y1, double x2, double y2)
  {
    double dtheta, theta1, theta2;

    theta1 = atan2(y1,x1);
    theta2 = atan2(y2,x2);
    dtheta = theta2 - theta1;
    while (dtheta > M_PI)
      dtheta -= 2*M_PI; //TWOPI;
    while (dtheta < -M_PI)
      dtheta += 2*M_PI; //TWOPI;

    return(dtheta);
  }  

  bool contains(const oriented_polygon& poly, point p)
  {
    double angle = 0;
    point p1, p2;
    int n = poly.vertices.size();

    for (int i = 0; i < n; i++) {
      p1.x = poly.pt(i).x - p.x;
      p1.y = poly.pt(i).y - p.y;
      p2.x = poly.pt((i+1)%n).x - p.x;
      p2.y = poly.pt((i+1)%n).y - p.y;
      angle += angle_2d(p1.x, p1.y, p2.x, p2.y);
    }

    if (abs(angle) < M_PI)
      { return false; }
    else
      { return true; }
  }

  bool is_horizontal(const oriented_polygon& p) {
    return within_eps(p.normal.z, 1.0, 0.001);
  }
  
}
