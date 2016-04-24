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

int main(int argc, char* argv[]) {
  assert(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto triangles = parse_stl(argv[1]).triangles;
  select_visible_triangles(triangles);

  double tool_radius = 0.05;
  double cut_depth = 0.1;
  double workpiece_height = 0.8;

  auto pockets = make_pockets(triangles, workpiece_height);
  cout << "# of pockets = " << pockets.size() << endl;
  // if (!(pockets.size() == 1)) {
  //   cout << "# of pockets = " << pockets.size() << endl;
  //   assert(false);
  // }
  auto lines = rough_pockets(pockets,
			     tool_radius,
			     cut_depth);
  // auto lines = mill_surface_lines(triangles,
  // 				  tool_radius,
  // 				  cut_depth,
  // 				  workpiece_height);

  point shift(0, 0, 0); //(-1, -1, 0); //-3, -2.5, -0.5);
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

