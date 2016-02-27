#include <cassert>
#include <iostream>

#include "checkers/bounds_checker.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/dxf_reader.h"

using namespace gca;

int main(int argc, char** argv) {
  assert(argc == 1);
  
  arena_allocator a;
  set_system_allocator(&a);

  cut_params params;
  params.default_feedrate = 30;
  params.set_default_feedrate = true;
  params.material_depth = 0.09;
  params.cut_depth = 0.05;
  params.safe_height = 0.45;
  params.machine_z_zero = -0.58;
  params.start_loc = point(5, 5, 0);
  params.start_orient = point(0, -1, 0);
  params.tools = DRAG_KNIFE_ONLY;
  params.target_machine = PROBOTIX_V90_MK2_VFD;

  vector<cut*> lines;
  lines.push_back(linear_cut::make(point(5, 5, 0), point(5, 7, 0)));
  lines.push_back(linear_cut::make(point(5, 7, 0), point(7, 7, 0)));
  lines.push_back(linear_cut::make(point(7, 7, 0), point(7, 5, 0)));
  lines.push_back(linear_cut::make(point(7, 5, 0), point(5, 5, 0)));
  vector<hole_punch*> holes;
  vector<b_spline*> splines;
  shape_layout l(lines, holes, splines);
  gprog* p = shape_layout_to_gcode(l, params);
  
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
