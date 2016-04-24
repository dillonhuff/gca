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

  double tool_radius = 0.025;
  double cut_depth = 0.1;
  double workpiece_height = 0.3;
  auto lines = mill_surface_lines(triangles,
				  tool_radius,
				  cut_depth,
				  workpiece_height);
  point shift(0, 0, 0); //(-1, -1, 0); //-3, -2.5, -0.5);
  vector<polyline> shifted_lines;
  for (auto l : lines) {
    vector<point> pts;
    for (auto pt : l) {
      pts.push_back(pt + shift);
    }
    shifted_lines.push_back(pts);
  }
  // vector<polyline> compressed_lines;
  // for (auto l : shifted_lines) {
  //   compressed_lines.push_back(compress_lines(l, 0.001));
  // }
  auto bs = emco_f1_code(shifted_lines);

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  cout << bs << endl;
}

