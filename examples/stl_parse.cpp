#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/axis_3.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/visual_debug.h"
#include "utils/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

void print_programs(const fabrication_plan& fix_plan) {
  cout << "Programs" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);

  for (auto f : fix_plan.steps()) {
    auto program = f.gcode_for_toolpaths(emco_f1_code);
    cout << program.name << endl;
    cout << program.blocks << endl;
  }

}

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto box_triangles = parse_stl(argv[1]).triangles;
  auto mesh = make_mesh(box_triangles, 0.001);

  // vice v = large_jaw_vice(4.0, point(-2, 0, 3));
  // std::vector<plate_height> parallel_plates{0.5};
  // fixtures fixes(v, parallel_plates);

  // tool t1(0.30, 3.0, 2, HSS, FLAT_NOSE);
  // tool t2(0.14, 3.15, 2, HSS, FLAT_NOSE);
  // vector<tool> tools{t1, t2};
  // workpiece workpiece_dims(4.0, 4.0, 4.0, ALUMINUM);

  vice test_vice = current_setup();
  std::vector<plate_height> parallel_plates{0.5};
  fixtures fixes(test_vice, parallel_plates);

  tool t1(0.30, 3.0, 2, HSS, FLAT_NOSE);
  tool t2(0.14, 3.15, 2, HSS, FLAT_NOSE);
  vector<tool> tools{t1, t2};

  workpiece workpiece_dims(1.5, 1.5, 1.5, ALUMINUM);
  
  fabrication_plan plan =
    make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

  for (auto step : plan.steps()) {
    visual_debug(step);
  }

  cout << "Custom fixtures" << endl;
  for (auto fix_plan : plan.custom_fixtures()) {
    print_programs(*fix_plan);
  }
  print_programs(plan);
}

