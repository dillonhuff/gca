#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/axis_3.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

bool is_vertical(const triangle t, double tolerance) {
  return within_eps(t.normal.z, 0.0, tolerance);
}

vector<polyline> finish_points(vector<triangle>& triangles,
			       double tolerance) {
  // delete_if(triangles,
  // 	    [tolerance](const triangle t)
  // 	    { return !is_vertical(t, tolerance); });
  stable_sort(begin(triangles), end(triangles),
	      [](const triangle l, const triangle r)
	      { return l.v1.x > r.v1.x; });
  stable_sort(begin(triangles), end(triangles),
	      [](const triangle l, const triangle r)
	      { return l.v1.y > r.v1.y; });
  stable_sort(begin(triangles), end(triangles),
	      [](const triangle l, const triangle r)
	      { return l.v1.z > r.v1.z; });
  vector<point> pts;
  for (auto t : triangles) {
    pts.push_back(t.v1 + -0.1 * (t.normal.normalize()));
  }
  return vector<polyline>({pts});
}

int main(int argc, char* argv[]) {
  assert(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto triangles = parse_stl(argv[1]).triangles;

  auto lines = mill_surface_lines(triangles, 0.2);
  point shift(-3, -2.5, -0.5);
  vector<polyline> shifted_lines;
  for (auto l : lines) {
    vector<point> pts;
    for (auto pt : l) {
      pts.push_back(pt + shift);
    }
    shifted_lines.push_back(pts);
  }
  auto bs = emco_f1_code(shifted_lines);

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  cout << bs << endl;
}

