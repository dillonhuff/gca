#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "geometry/point.h"
#include "geometry/polygon.h"
#include "geometry/polyline.h"
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

vector<cut*> dummy_cuts(const polyline& p) {
  auto ls = p.lines();
  assert(ls.size() > 0);
  vector<cut*> c;
  for (auto l : ls) {
    c.push_back(mk_cut(l.start, l.end));
  }
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

vector<oriented_polygon> preprocess_triangles(const vector<triangle> tris) {
  auto triangles = tris;
  delete_if(triangles, [](const triangle& t) { return !is_upward_facing(t, 1e-2); });
  greedy_adjacent_chains(triangles.begin(), triangles.end(),
			 [](const triangle& x,
			    const triangle& y)
			 { return same_orientation(x, y, 0.01); });
  vector<vector<triangle>> constant_orientation_groups;
  split_by(triangles, constant_orientation_groups,
	   [](const triangle& x,
	      const triangle& y)
	   { return same_orientation(x, y, 0.01); });
  vector<oriented_polygon> polys;
  for (auto g : constant_orientation_groups) {
    assert(g.size() > 0);
    auto merged_polys = merge_triangles(g);
    polys.insert(polys.end(), merged_polys.begin(), merged_polys.end());
  }
  return polys;
}

void sanity_check_pockets(const vector<oriented_polygon>& pockets) {
  for (auto p : pockets) {
    assert(is_horizontal(p));
  }
}

// TODO: Add depth sorting
void order_pockets(const vector<oriented_polygon>& pockets) {
  
}

vector<polyline> pocket_lines(const oriented_polygon& pocket,
			      double workpiece_height,
			      double cut_depth) {
  vector<polyline> ps;
  return ps;
}

vector<polyline> compute_pocket_paths(const vector<oriented_polygon>& pockets,
				      const box workpiece,
				      const cut_params& params) {
  vector<polyline> paths;
  double workpiece_height = workpiece.z_max;
  for (auto pocket : pockets) {
    auto pocket_paths = pocket_lines(pocket, workpiece_height, params.cut_depth);
    paths.insert(paths.end(), pocket_paths.begin(), pocket_paths.end());
  }
  return paths;
}

vector<cut*> generate_toolpath(const vector<oriented_polygon>& pockets,
			       const box workpiece,
			       const cut_params& params) {
  sanity_check_pockets(pockets);
  order_pockets(pockets);
  auto depth_pockets = compute_pocket_paths(pockets, workpiece, params);
  vector<cut*> cuts;
  for (auto p : depth_pockets) {
    auto cs = dummy_cuts(p);
    cuts.insert(cuts.end(), cs.begin(), cs.end());
  }
  return cuts;
}

box make_box(point bottom_left, double x_len, double y_len, double z_len) {
  return box(bottom_left.x, bottom_left.x + x_len,
	     bottom_left.y, bottom_left.y + y_len,
	     bottom_left.z, bottom_left.z + z_len);
}

int main(int argc, char* argv[]) {
  arena_allocator a;
  set_system_allocator(&a);

  assert(argc == 2);
  string stl_path = argv[1];
  auto info = parse_stl(stl_path);
  cout << "# triangles: " << info.triangles.size() << endl;
  // box workpiece = make_box(point(0, 0, 0), 10.0, 10.0, 1.9);
  // auto polys = preprocess_triangles(info.triangles);
  // cut_params params;
  // params.target_machine = EMCO_F1;
  // params.safe_height = 2.0;
  // params.cut_depth = 0.5;
  // // TODO: Find a better way to do this
  // params.material_depth = workpiece.z_max;
  // auto cuts = generate_toolpath(polys, workpiece, params);
  // auto bs = cuts_to_gcode(cuts, params);
  // cout << bs << endl;
}
