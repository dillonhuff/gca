#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/axis_3.h"
#include "synthesis/mesh_to_gcode.h"
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

  auto box_triangles = parse_stl(argv[1]).triangles;
  auto mesh = make_mesh(box_triangles, 0.001);

  vice v = current_setup();
  tool t1(0.30, 3.0, FLAT_NOSE);
  tool t2(0.14, 3.2, FLAT_NOSE);
  vector<tool> tools{t1, t2};
  workpiece workpiece_dims(1.9, 1.87, 1.9);
  auto result_programs = mesh_to_gcode(mesh, v, tools, workpiece_dims);

  cout << "All programs" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  for (auto program : result_programs) {
    cout << program.name << endl;
    cout << program.blocks << endl;
  }
}

