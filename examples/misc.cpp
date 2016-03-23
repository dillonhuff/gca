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
  params.material_depth = 0.075;
  params.push_depth = 0.00;
  params.cut_depth = 0.05;
  params.safe_height = 0.45;
  params.machine_z_zero = -0.58;
  params.start_loc = point(5, 5, 0);
  params.start_orient = point(0, -1, 0);
  params.tools = DRAG_KNIFE_ONLY;
  params.target_machine = PROBOTIX_V90_MK2_VFD;

  vector<cut*> lines;
  lines.push_back(circular_arc::make(point(6, 6, 0),
				     point(8, 6, 0),
				     point(1, 0, 0),
				     CLOCKWISE,
				     XY));
  vector<hole_punch*> holes;
  vector<b_spline*> splines;
  shape_layout l(lines, holes, splines);

  vector<block> s = shape_layout_to_gcode(l, params);
  
  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  cout << s << endl;

  int num_warns = check_bounds(s, GCA_ABSOLUTE,
  			       0.5, 17,			       
  			       1.2, 13.2,
  			       -4.1, -0.05);
  if (num_warns > 0) {
    cout << "Num warnings: " << num_warns << endl;
  }
  return 0;
}
