#include "geometry/triangular_mesh.h"
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

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto box_triangles = parse_stl(argv[1]).triangles;
  auto mesh = make_mesh(box_triangles, 0.001);

  vice test_vice = current_setup(); //large_jaw_vice(15, point(0, 0, 0)); //current_setup();
  std::vector<plate_height> parallel_plates{0.5};
  fixtures fixes(test_vice, parallel_plates);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(0.25);
  t1.set_cut_length(0.6);

  t1.set_shank_diameter(3.0 / 8.0);
  t1.set_shank_length(0.1);

  t1.set_holder_diameter(2.5);
  t1.set_holder_length(3.5);
    
  tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(0.5);
  t2.set_cut_length(0.3);

  t2.set_shank_diameter(0.5);
  t2.set_shank_length(0.0);

  t2.set_holder_diameter(2.5);
  t2.set_holder_length(3.5);
  
  vector<tool> tools{t1, t2};

  workpiece workpiece_dims(2, 2, 2.7, ALUMINUM);
  
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

