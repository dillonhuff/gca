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
#include "synthesis/toolpath_generation.h"
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

polyline extract_part_base_outline(const vector<triangle>& tris) {
  auto triangles = tris;
  delete_if(triangles, [](const triangle& t)
	    { return !within_eps(t.normal, point(0, 0, -1), 1e-2); });
  auto outlines = merge_triangles(triangles);
  assert(outlines.size() == 1);
  auto base_outline = outlines.front();
  vector<point> vertices = base_outline.vertices;
  vertices.push_back(vertices.front());
  return polyline(vertices);
}

vector<oriented_polygon> preprocess_triangles(const vector<triangle>& tris) {
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

// TODO: Move to box.h
box make_box(point bottom_left, double x_len, double y_len, double z_len) {
  return box(bottom_left.x, bottom_left.x + x_len,
	     bottom_left.y, bottom_left.y + y_len,
	     bottom_left.z, bottom_left.z + z_len);
}

int main(int argc, char* argv[]) {
  arena_allocator a;
  set_system_allocator(&a);

  assert(argc == 2);
  auto info = parse_stl(argv[1]);
  vector<triangle> triangles = info.triangles;
  auto outline = extract_part_base_outline(triangles);

  for (auto p : outline) {
    cout << p << endl;
  }

  double inc = 0.1;
  double deg = 90;
  int num_phases = 2;
  double start_depth = 1.2;
  double end_depth = 0.03;
  double cut_depth = 0.35;

  pocket_info_2P5D pocket(outline, inc, deg, num_phases, start_depth, end_depth, cut_depth);
  auto pocket_lines = pocket_2P5D_lines(pocket);

  vector<cut*> cuts;
  for (auto p : pocket_lines) {
    auto cs = dummy_cuts(p);
    cuts.insert(cuts.end(), cs.begin(), cs.end());
  }

  cout << "# cuts: " << cuts.size() << endl;

  cut_params params;
  params.target_machine = EMCO_F1;
  params.safe_height = 20.0;
  params.cut_depth = 0.5;
  params.material_depth = 1.5;

  auto bs = cuts_to_gcode(cuts, params);
  cout << bs << endl;

}