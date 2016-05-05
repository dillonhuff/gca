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
    if (within_eps(d, 0)) { return 90.0; }
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

  double dot(point u, point v) {
    return u.dot(v);
  }

  point cross(point b, point c) {
    double x = (b).y * (c).z - (c).y * (b).z;
    double y = (b).z * (c).x - (c).z * (b).x;
    double z = (b).x * (c).y - (c).x * (b).y;
    return point(x, y, z);
  }

  point project_onto(point p, point proj_d) {
    point proj_dir = proj_d.normalize();
    return (p.dot(proj_dir))*proj_dir;
  }

  double signed_distance_along(const point p, const point proj_dir) {
    point dir = proj_dir.normalize();
    double len = p.dot(dir);
    return len;
  }

  double greater_than_diameter(const point normal,
			       const std::vector<point>& centroids) {
    vector<point> face_projections(centroids.size());
    transform(begin(centroids), end(centroids),
	      begin(face_projections),
	      [normal](const point cent) {
		return project_onto(cent, normal);
	      });
    auto max_e = max_element(begin(face_projections), end(face_projections),
			     [](const point l, const point r)
			     { return l.len() < r.len(); });
    double ray_len = 2*(*max_e).len();
    return ray_len;
  }

  double diameter(const point normal,
		  const std::vector<point>& pts) {
    vector<double> face_projections(pts.size());
    transform(begin(pts), end(pts),
	      begin(face_projections),
	      [normal](const point cent) {
		return signed_distance_along(cent, normal);
	      });
    auto max_e = max_element(begin(face_projections), end(face_projections));
    auto min_e = min_element(begin(face_projections), end(face_projections));
    return abs(*max_e - *min_e);
  }

  double max_distance_along(const std::vector<point>& pts, const point normal) {
    vector<double> face_projections(pts.size());
    transform(begin(pts), end(pts),
	      begin(face_projections),
	      [normal](const point cent) {
		return signed_distance_along(cent, normal);
	      });
    auto max_e = max_element(begin(face_projections), end(face_projections));
    return *max_e;
  }

  double min_distance_along(const std::vector<point>& pts, const point normal) {
    vector<double> face_projections(pts.size());
    transform(begin(pts), end(pts),
	      begin(face_projections),
	      [normal](const point cent) {
		return signed_distance_along(cent, normal);
	      });
    auto min_e = min_element(begin(face_projections), end(face_projections));
    return *min_e;
  }
  

}
