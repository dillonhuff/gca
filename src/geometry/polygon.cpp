#include <cmath>

#include "geometry/polygon.h"
#include "geometry/polyline.h"

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

  bool contains(const oriented_polygon& poly,
		const oriented_polygon& maybe_contained) {
    for (auto pt : maybe_contained.vertices) {
      if (!contains(poly, pt)) {
	return false;
      }
    }
    return true;
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

  box bounding_box(const oriented_polygon& p) {
    return bound_positions(p.vertices);
  }

  bool overlaps(line l, const oriented_polygon& p) {
    polyline pl(p.vertices);
    for (auto pll : pl.lines()) {
      if (segment_intersection_2d(l, pll).just)
	{ return true; }
    }
    return false;
  }

  template<typename InputIt>
  bool overlaps_any(line l, InputIt s, InputIt e) {
    while (s != e) {
      if (overlaps(l, *s)) {
	return true;
      }
      ++s;
    }
    return false;
  }

  oriented_polygon exterior_offset(const oriented_polygon& p,
				   double inc) {
    auto vs = p.vertices;
    vs.push_back(p.vertices.front());
    polyline pl(vs);
    auto off_l = offset(pl, exterior_direction(pl), inc);
    vector<point> pts(begin(off_l), end(off_l));
    return oriented_polygon(p.normal, pts);
  }

  oriented_polygon interior_offset(const oriented_polygon& p,
				   double inc) {
    auto vs = p.vertices;
    vs.push_back(p.vertices.front());
    polyline pl(vs);
    auto off_l = offset(pl, interior_direction(pl), inc);
    vector<point> pts(begin(off_l), end(off_l));
    return oriented_polygon(p.normal, pts);
  }

}
