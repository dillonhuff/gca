#include "geometry/mesh_operations.h"
#include "process_planning/axis_location.h"
#include "synthesis/mesh_to_gcode.h"
#include "system/parse_stl.h"
#include "utils/algorithm.h"
#include "utils/arena_allocator.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto mesh = parse_stl(argv[1], 0.001);

  box bounding = mesh.bounding_box();

  point axis = part_axis(mesh);
  cout << "Axis = " << axis << endl;

  cout << "Diameter along axis = " << diameter(axis, mesh) << endl;

  cout << "Bounding box = " << endl;
  cout << bounding << endl;

  double max_dim = bounding.z_len();
  double desired_dim = 0.9;
  double scale_factor = desired_dim / max_dim;

  cout << "Scale factor = " << scale_factor << endl;

  auto scale_func = [scale_factor](const point p) {
    return scale_factor*p;
  };

  triangular_mesh scaled_mesh =
    mesh.apply_to_vertices(scale_func);

  box scaled_bounding = scaled_mesh.bounding_box();
  cout << "Scaled bounding box" << endl;
  cout << scaled_bounding << endl;

  vice test_vice = current_setup(); //large_jaw_vice(15, point(0, 0, 0)); //current_setup();
  std::vector<plate_height> parallel_plates{0.5};
  fixtures fixes(test_vice, parallel_plates);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(0.25);
  t1.set_cut_length(0.4);

  t1.set_shank_diameter(3.0 / 8.0);
  t1.set_shank_length(0.1);

  t1.set_holder_diameter(2.5);
  t1.set_holder_length(3.5);
    
  tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(0.5);
  t2.set_cut_length(0.3);

  t2.set_shank_diameter(0.3 / 8.0);
  t2.set_shank_length(0.01);

  t2.set_holder_diameter(2.5);
  t2.set_holder_length(3.5);
  
  tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
  t3.set_cut_diameter(0.2334);
  t3.set_cut_length(1.2);

  t3.set_shank_diameter(0.5);
  t3.set_shank_length(0.05);

  t3.set_holder_diameter(2.5);
  t3.set_holder_length(3.5);
  
  vector<tool> tools{t1, t2, t3};

  workpiece workpiece_dims(1.5, 1.5, 1.5, ALUMINUM);

  fabrication_plan plan =
    make_fabrication_plan(scaled_mesh, fixes, tools, {workpiece_dims});

  cout << "Custom fixtures" << endl;
  for (auto fix_plan : plan.custom_fixtures()) {
    print_programs(*fix_plan);
  }

  print_programs(plan);
  
}

