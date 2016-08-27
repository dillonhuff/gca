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

  point cross(point u, point v) {
    double x = u.y * v.z - v.y * u.z;
    double y = u.z * v.x - v.z * u.x;
    double z = u.x * v.y - v.x * u.y;
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

  point max_along(const std::vector<point>& pts, const point normal) {
    assert(pts.size() > 0);
    auto max_e = max_element(begin(pts), end(pts),
			     [normal](const point l, const point r)
			     { return signed_distance_along(l, normal) <
			       signed_distance_along(r, normal); });
    return *max_e;
  }

  point min_along(const std::vector<point>& pts, const point normal) {
    assert(pts.size() > 0);
    auto min_e = min_element(begin(pts), end(pts),
			     [normal](const point l, const point r)
			     { return signed_distance_along(l, normal) <
			       signed_distance_along(r, normal); });
    return *min_e;
  }
  
  double max_distance_along(const std::vector<point>& pts, const point normal) {
    assert(pts.size() > 0);
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

  std::vector<point> shift(const point s, const std::vector<point>& pts) {
    vector<point> res;
    for (auto p : pts) {
      res.push_back(p + s);
    }
    return res;
  }
  

  bool components_within_eps(const point l, const point r, const double tol) {
    return within_eps(l.x, r.x, tol) &&
      within_eps(l.y, r.y, tol) &&
      within_eps(l.z, r.z, tol);
  }

  bool no_duplicate_points(const std::vector<point>& pts, const double tol) {
    for (unsigned i = 0; i < pts.size(); i++) {
      point p = pts[i];

      for (unsigned j = 0; j < pts.size(); j++) {
	if (i != j) {
	  point q = pts[j];

	  if (components_within_eps(p, q, tol)) {
	    for (unsigned k = 0; k < pts.size(); k++) {
	      cout << "pts[" << k << "] = " << pts[k] << endl;
	    }

	    cout << "DONE WITH POINTS" << endl;
	    
	    cout << "# of points = " << pts.size() << endl;
	    cout << "duplicate points at i = " << i << " , j = " << j << endl;
	    cout << "pts[i] = " << p << endl;
	    cout << "pts[j] = " << q << endl;
	    return false;
	  }
	}
      }
    }

    return true;
  }

}
