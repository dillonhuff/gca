#include <cassert>

#include "geometry/line.h"
#include "geometry/triangle.h"
#include "system/algorithm.h"

namespace gca {

  double min_z(const vector<triangle>& triangles) {
    auto t = *min_element(begin(triangles), end(triangles),
			  [](const triangle l, const triangle r)
			  { return l.v1.z < r.v1.z; });
    return t.v1.z;
  }

  bool is_upward_facing(const triangle& t, double tolerance) {
    return (t.normal.normalize()).z > tolerance;
  }

  bool same_orientation(const triangle& x, const triangle& y, double tolerance) {
    return within_eps((x.normal - y.normal).len(), 0.0, tolerance);
  }

  vector<point> collect_polygon(vector<line>& lines) {
    assert(lines.size() > 0);
    vector<point> points;
    vector<line> to_remove;
    points.push_back(lines.front().start);
    points.push_back(lines.front().end);
    lines.erase(lines.begin());
    unsigned i = 0;
    while (lines.size() > 0 && i < lines.size()) {
      if (within_eps(lines[i].start, points.back())) {
	if (within_eps(lines[i].end, points.front())) {
	  lines.erase(lines.begin() + i);	  
	  return points;
	}
	points.push_back(lines[i].end);
	lines.erase(lines.begin() + i);
	i = 0;
      } else if (within_eps(lines[i].end, points.back())) {
	if (within_eps(lines[i].start, points.front())) {
	  lines.erase(lines.begin() + i);
	  return points;
	}
	points.push_back(lines[i].start);
	lines.erase(lines.begin() + i);
	i = 0;
      } else {
	i++;
      }
    }
    return points;
  }

  vector<oriented_polygon>
  unordered_segments_to_polygons(point normal,
				 vector<line>& lines) {
    vector<oriented_polygon> ps;
    auto ls = lines;
    while (ls.size() > 0) {
      vector<point> vertices = collect_polygon(ls);
      ps.push_back(oriented_polygon(normal, vertices));
    }
    return ps;
  }

  vector<oriented_polygon> mesh_bounds(const vector<triangle>& triangles) {
    auto tris = triangles;
    vector<oriented_polygon> ps;
    if (tris.size() == 0) {
      return ps;
    }
    point normal = triangles.front().normal;
    vector<line> tri_lines;
    for (auto t : triangles) {
      tri_lines.push_back(line(t.v1, t.v2));
      tri_lines.push_back(line(t.v2, t.v3));
      tri_lines.push_back(line(t.v3, t.v1));
    }
    vector<line> no_dups;
    for (auto l : tri_lines) {
      if (count_in(l, tri_lines) == 1) {
	no_dups.push_back(l);
      }
    }
    return unordered_segments_to_polygons(normal, no_dups);
  }

  ostream& operator<<(ostream& out, const triangle& t) {
    cout << "---- TRIANGLE ----" << endl;
    cout << t.normal << endl;
    cout << t.v1 << endl;
    cout << t.v2 << endl;
    cout << t.v3 << endl;
    return out;
  }

  bool in_projection(const triangle t, const point p) {
    vector<point> vs{t.v1, t.v2, t.v3, t.v1};
    return contains(oriented_polygon(t.normal, vs), p);
  }

  double z_at(const triangle t, double x, double y) {

    point a = t.v1;
    point b = t.v2;
    point c = t.v3;
    double y_c_numerator = (b.x - a.x)*(c.z - a.z) - (c.x - a.x)*(b.z - a.z);
    double x_c_numerator = (b.y - a.y)*(c.z - a.z) - (c.y - a.y)*(b.z - a.z);
    double denom = (b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y);
    double y_c = y_c_numerator / denom;
    double x_c = x_c_numerator / denom;
    return a.z + y_c*(y - a.y) - x_c*(x - a.x);
  }

  bool below(const triangle t, const point p) {
    return p.z < z_at(t, p.x, p.y);
  }

  bool ray_intersects_triangle(point p, point d,
			       point v0, point v1, point v2) {
    point e1, e2, s, q;
    double a, f, u, v;
    e1 = v1 - v0;
    e2 = v2 - v0;

    point h = cross(d, e2);
    a = dot(e1, h);

    if (a > -0.00001 && a < 0.00001) {
      return(false);
    }

    f = 1/a;
    s = p - v0;
    u = f * (dot(s, h));

    if (u < 0.0 || u > 1.0)
      return(false);

    q = cross(s, e1);
    v = f * dot(d, q);

    if (v < 0.0 || u + v > 1.0) {
      return false;
    }

    // at this stage we can compute t to find out where
    // the intersection point is on the line
    double t = f * dot(e2, q);

    if (t > 0.00001) {// ray intersection
      return true;
    } else {// this means that there is a line intersection
      // but not a ray intersection
      return false;
    }

  }  

  bool intersects(const triangle t, const line l) {
    return ray_intersects_triangle(l.start, l.end - l.start,
    				   t.v1, t.v2, t.v3) &&
      ray_intersects_triangle(l.end, l.start - l.end,
			      t.v1, t.v2, t.v3);
  }

  void select_visible_triangles(vector<triangle>& triangles) {
    delete_if(triangles,
	      [](const triangle t)
	      { return !is_upward_facing(t, 0.01); });
  }

  bool intersects_triangles(line l, const vector<triangle>& triangles) {
    for (auto t : triangles) {
      if (intersects(t, l)) { return true; }
    }
    return false;
  }

  triangle apply(const matrix<3, 3> m, const triangle& t) {
    return triangle(m*t.normal,
		    m*t.v1,
		    m*t.v2,
		    m*t.v3);
  }

  double distance_along(point normal, const triangle t) {
    point p = t.v1;
    point dir = normal.normalize();
    return ((p.dot(dir))*dir).len();
  }

}
