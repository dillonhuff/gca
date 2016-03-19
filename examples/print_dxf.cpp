#include <cassert>
#include <dirent.h>
#include <iostream>

#include "checkers/bounds_checker.h"
#include "core/parser.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/dxf_reader.h"

using namespace gca;

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: print-dxf <gcode file path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  cut_params params;
  params.default_feedrate = 30;
  params.set_default_feedrate = true;
  params.material_depth = 0.075;
  params.push_depth = 0.00;
  params.cut_depth = 0.05;
  params.safe_height = 0.45;
  params.machine_z_zero = -1.904;
  params.start_loc = point(5, 5, 0);
  params.start_orient = point(0, -1, 0);
  params.tools = DRAG_KNIFE_ONLY;
  params.target_machine = PROBOTIX_V90_MK2_VFD;

  auto l = read_dxf(argv[1]);
  auto p = parse_gprog(shape_layout_to_gcode_string(l, params));

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  p->print_nc_output(cout);

  int num_warns = check_bounds(p, GCA_ABSOLUTE,
			       0.5, 17,			       
			       1.2, 13.2,
			       -4.1, -0.05);
  if (num_warns > 0) {
    cout << "Num warnings: " << num_warns << endl;
  }
  return 0;
}
