#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/axis_3.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "utils/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {
  assert(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto box_triangles = parse_stl(argv[1]).triangles;
  auto mesh = make_mesh(box_triangles, 0.001);

  vice v = current_setup();
  std::vector<plate_height> plates; //{0.1, 0.3};
  std::vector<plate_height> parallel_plates{0.5};
  fixtures fixes(v, plates, parallel_plates);

  //  tool t1(0.30, 3.0, 2, HSS, FLAT_NOSE);
  tool t2(0.14, 3.15, 2, HSS, FLAT_NOSE);
  vector<tool> tools{t2}; //, t2};
  workpiece workpiece_dims(2.0, 1.54, 1.6, ACETAL);
  auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);

  cout << "All programs" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  for (auto program : result_programs) {
    cout << program.name << endl;
    cout << program.blocks << endl;
  }
}

