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

  vice test_vice = emco_vice(point(-0.8, -4.4, -3.3)); //current_setup(); //emco_vice(point(1.0, 1.0, 1.0));
  tool t1(0.3, FLAT_NOSE);
  vector<tool> tools{t1};
  workpiece workpiece_dims(1.5, 1.2, 1.5);
  auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);

  cout << "All programs" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  for (auto program : result_programs) {
    cout << program.name << endl;
    cout << program.blocks << endl;
  }
}

