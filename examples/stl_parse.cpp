#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

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

vector<block> generate_mill_paths(const polyline& outline,
				  vector<vector<triangle>>& surfaces) {
  double start_depth = surfaces.front().front().v1.z;
  double end_depth = outline.front().z;

  pocket_info_2P5D pocket(outline, start_depth, end_depth);

  auto pocket_lines = pocket_2P5D_lines(pocket);

  vector<cut*> cuts;
  for (auto p : pocket_lines) {
    auto cs = polyline_cuts(p);
    cuts.insert(cuts.end(), cs.begin(), cs.end());
  }

  cout << "# cuts: " << cuts.size() << endl;

  cut_params params;
  params.target_machine = EMCO_F1;
  params.safe_height = start_depth + 0.05;

  return cuts_to_gcode(cuts, params);
}

int main(int argc, char* argv[]) {
  arena_allocator a;
  set_system_allocator(&a);

  assert(argc == 2);

  auto info = parse_stl(argv[1]);
  vector<triangle> triangles = info.triangles;
  auto outline = extract_part_base_outline(triangles);
  auto surfaces = millable_surfaces(triangles);
  stable_sort(begin(surfaces), end(surfaces),
	      [](const vector<triangle>& tsl,
		 const vector<triangle>& tsr)
	      { return tsl.front().v1.z > tsr.front().v1.z; });

  auto bs = generate_mill_paths(outline, surfaces);

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  cout << bs << endl;
}
