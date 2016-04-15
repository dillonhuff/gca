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

  void extract_millable_surface(vector<triangle>& triangles,
				vector<vector<triangle>>& millable_surfaces) {
    assert(triangles.size() > 0);
    vector<triangle> ts;
    ts.push_back(triangles.back());
    triangles.pop_back();
    millable_surfaces.push_back(ts);
  }

  vector<vector<triangle>> millable_surfaces(const vector<triangle>& tris) {
    auto triangles = tris;
    delete_if(triangles, [](const triangle& t)
	      { return !is_upward_facing(t, 1e-2); });
    vector<vector<triangle>> millable_surfaces;
    stable_sort(begin(triangles), end(triangles),
		[](const triangle l, const triangle r)
		{ return l.v1.z < r.v1.z; });
    split_by(triangles, millable_surfaces,
    	     [](const triangle l, const triangle r)
    	     { return within_eps(l.v1.z, r.v1.z); });
    return millable_surfaces;
  }

}
