#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/shapes_to_gcode.h"
#include "backend/toolpath_generation.h"
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

  // vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
  // std::vector<plate_height> parallel_plates{0.5, 0.7};
  // fixtures fixes(test_vice, parallel_plates);

  // tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
  // t1.set_cut_diameter(0.1);
  // t1.set_cut_length(0.4);

  // t1.set_shank_diameter(3.0 / 8.0);
  // t1.set_shank_length(0.1);

  // t1.set_holder_diameter(2.0);
  // t1.set_holder_length(2.5);

  // tool t2(0.12, 3.0, 4, HSS, FLAT_NOSE);
  // t2.set_cut_diameter(0.12);
  // t2.set_cut_length(1.2);

  // t2.set_shank_diameter(0.1);
  // t2.set_shank_length(0.5);

  // t2.set_holder_diameter(2.0);
  // t2.set_holder_length(2.5);
    
  // vector<tool> tools{t1, t2};

  // workpiece workpiece_dims(3.0, 1.9, 3.0, ACETAL);
  
  // vice test_vice = current_setup(); //large_jaw_vice(15, point(0, 0, 0)); //current_setup();
  // test_vice = top_jaw_origin_vice(test_vice);

  // std::vector<plate_height> parallel_plates{0.5};
  // fixtures fixes(test_vice, parallel_plates);

  // tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  // t1.set_cut_diameter(0.25);
  // t1.set_cut_length(0.6);

  // t1.set_shank_diameter(3.0 / 8.0);
  // t1.set_shank_length(0.1);

  // t1.set_holder_diameter(2.5);
  // t1.set_holder_length(3.5);
    
  // tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
  // t2.set_cut_diameter(0.5);
  // t2.set_cut_length(0.3);

  // t2.set_shank_diameter(0.5);
  // t2.set_shank_length(0.0);

  // t2.set_holder_diameter(2.5);
  // t2.set_holder_length(3.5);

  // tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
  // t3.set_cut_diameter(0.12); //0.2334);
  // t3.set_cut_length(1.2);

  // t3.set_shank_diameter(0.5);
  // t3.set_shank_length(0.05);

  // t3.set_holder_diameter(2.5);
  // t3.set_holder_length(3.5);
  // vector<tool> tools{t1, t2, t3};

  // workpiece workpiece_dims(2, 2, 2.7, ALUMINUM);

  vice test_v = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0));
  vice test_vice = top_jaw_origin_vice(test_v);

  std::vector<plate_height> plates{0.1, 0.3, 0.7};
  fixtures fixes(test_vice, plates);

  workpiece workpiece_dims(4.0, 4.0, 4.0, ALUMINUM);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(0.25);
  t1.set_cut_length(0.6);

  t1.set_shank_diameter(3.0 / 8.0);
  t1.set_shank_length(0.3);

  t1.set_holder_diameter(2.5);
  t1.set_holder_length(3.5);
    
  tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(0.5);
  t2.set_cut_length(0.3);

  t2.set_shank_diameter(0.5);
  t2.set_shank_length(0.5);

  t2.set_holder_diameter(2.5);
  t2.set_holder_length(3.5);

  tool t3{0.125, 3.94, 4, HSS, FLAT_NOSE};
  t3.set_cut_diameter(0.125);
  t3.set_cut_length(1.2);

  t3.set_shank_diameter(0.5);
  t3.set_shank_length(0.05);

  t3.set_holder_diameter(2.5);
  t3.set_holder_length(3.5);

  tool t4{0.5, 3.94, 4, HSS, FLAT_NOSE};
  t4.set_cut_diameter(0.5);
  t4.set_cut_length(2.5);

  t4.set_shank_diameter(0.5);
  t4.set_shank_length(0.5);

  t4.set_holder_diameter(2.5);
  t4.set_holder_length(3.5);
    
  vector<tool> tools{t1, t2, t3, t4};
  
  fabrication_plan plan =
    make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

  for (auto step : plan.steps()) {
    cout << "# of toolpaths = " << step.toolpaths().size() << endl;
    visual_debug(step);
  }

  print_programs_wells_no_TLC(plan);
}

