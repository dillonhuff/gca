#include <cassert>

#include "geometry/line.h"
#include "geometry/triangle.h"
#include "system/algorithm.h"

namespace gca {

  bool is_upward_facing(const triangle& t, double tolerance) {
    return t.normal.z > tolerance;
  }

  bool same_orientation(const triangle& x, const triangle& y, double tolerance) {
    return within_eps((x.normal - y.normal).len(), 0.0, tolerance);
  }

  bool same_line(const line l, const line r) {
    bool ss = within_eps(l.start, r.start);
    bool ee = within_eps(l.end, r.end);
    bool se = within_eps(l.start, r.end);
    bool es = within_eps(l.end, r.start);
    return (ss && ee) || (se && es);
  }

  bool count_in(const line l, const vector<line> ls) {
    return count_if(ls.begin(), ls.end(), [l](const line r)
		    { return same_line(l, r); });
  }

  bool adj_segment(const line l, const line r) {
    return within_eps(l.start, r.end) ||
      within_eps(l.end, r.start);
  }

  vector<point> collect_polygon(vector<line>& lines) {
    assert(lines.size() > 0);
    vector<point> points;
    vector<line> to_remove;
    points.push_back(lines.front().start);
    points.push_back(lines.front().end);
    to_remove.push_back(lines.front());
    unsigned i = 1;
    while (i < lines.size()) {
      if (within_eps(lines[i].start, points.back())) {
	points.push_back(lines[i].end);
	to_remove.push_back(lines[i]);
      } else if (within_eps(lines[i].end, points.back())) {
	points.push_back(lines[i].start);
	to_remove.push_back(lines[i]);
      }
      i++;
    }
    auto should_remove = [&to_remove](const line l) {
      return find_if(to_remove.begin(), to_remove.end(),
		     [l](const line r)
      { return same_line(l, r); }) != to_remove.end();
    };
    delete_if(lines, should_remove);
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

  vector<oriented_polygon> merge_triangles(const vector<triangle>& triangles) {
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
      if (count_in(l, no_dups) == 0) {
	no_dups.push_back(l);
      }
    }
    return unordered_segments_to_polygons(normal, no_dups);
  }
  
  ostream& operator<<(ostream& out, const triangle& t) {
    cout << "---- TRIANGLE ----" << endl;
    cout << t.normal << endl;;
    cout << t.v1 << endl;;
    cout << t.v2 << endl;;
    cout << t.v3 << endl;;
    return out;
  }

}
