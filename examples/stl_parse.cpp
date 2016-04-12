#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "geometry/point.h"
#include "geometry/polygon.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"
#include "system/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int name_length = 80;

cut* mk_cut(const point l, const point r) {
  auto c = linear_cut::make(l, r);
  c->set_spindle_speed(lit::make(3000));
  c->set_feedrate(lit::make(10));
  return c;
}

vector<cut*> dummy_cuts(const oriented_polygon& p) {
  assert(p.vertices.size() >= 2);
  vector<cut*> c(p.vertices.size() - 1);
  apply_between(p.vertices.begin(), p.vertices.end(), c.begin(), mk_cut);
  return c;
}

point flip_yz(point r) {
  auto tmp = r.z;
  point p = r;
  p.z = r.y;
  p.y = tmp;
  return p;
}

triangle flip_triangle_yz(const triangle t) {
  point np = t.normal;
  point v1 = t.v1;
  point v2 = t.v2;
  point v3 = t.v3;
  return triangle(flip_yz(np), flip_yz(v1), flip_yz(v2), flip_yz(v3));
}

int main(int argc, char* argv[]) {
  arena_allocator a;
  set_system_allocator(&a);

  assert(argc == 2);
  string stl_path = argv[1];
  auto info = parse_stl(stl_path);
  auto ts = info.triangles;
  vector<triangle> triangles;
  for (auto t : ts) {
    triangles.push_back(flip_triangle_yz(t));
  }
  cout << "# triangles: " << triangles.size() << endl;
  delete_if(triangles, [](const triangle& t) { return !is_upward_facing(t, 1e-2); });
  cout << "# upward facing triangles: " << triangles.size() << endl;
  greedy_adjacent_chains(triangles.begin(), triangles.end(),
			 [](const triangle& x,
			    const triangle& y)
			 { return same_orientation(x, y, 0.01); });
  cout << "Grouped by adjacency" << endl;
  vector<vector<triangle>> constant_orientation_groups;
  split_by(triangles, constant_orientation_groups,
	   [](const triangle& x,
	      const triangle& y)
	   { return same_orientation(x, y, 0.01); });
  cout << "# constant orientation groups: " << constant_orientation_groups.size() << endl;
  vector<oriented_polygon> polys;
  for (auto g : constant_orientation_groups) {
    assert(g.size() > 0);
    auto merged_polys = merge_triangles(g);
    polys.insert(polys.end(), merged_polys.begin(), merged_polys.end());
  }
  cout << "# merged polygons: " << polys.size() << endl;
  vector<cut*> cuts;
  for (auto p : polys) {
    if (is_horizontal(p)) {
      auto cs = dummy_cuts(p);
      cuts.insert(cuts.end(), cs.begin(), cs.end());
    }
  }
  cut_params params;
  params.target_machine = EMCO_F1;
  params.safe_height = 2.0;
  auto bs = cuts_to_gcode(cuts, params);
  cout << bs << endl;
}
