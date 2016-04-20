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
  arena_allocator a;
  set_system_allocator(&a);

  assert(argc == 2);

  auto info = parse_stl(argv[1]);
  vector<triangle> triangles = info.triangles;
  auto lines = mill_surface_lines(triangles, 0.1);
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
