#include <cassert>
#include <iostream>

#include "checkers/bounds_checker.h"
#include "core/parser.h"
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
  params.material_depth = 0.099;
  params.push_depth = 0.00;
  params.cut_depth = 0.05;
  params.safe_height = 0.45;
  params.machine_z_zero = -1.90425;
  params.start_loc = point(9, 8, 0);
  params.start_orient = point(0, -1, 0);
  params.tools = DRAG_KNIFE_ONLY;
  params.target_machine = PROBOTIX_V90_MK2_VFD;

  vector<cut*> lines;
  // Add outer rectangle
  double d = 2.0;
  point bottom_left_corner = point(9.4, 4.7, 0);
  point p0 = bottom_left_corner;
  point p1 = p0 + point(0, d, 0);
  point p2 = p0  + point(d, d, 0);
  point p3 = p0 + point(d, 0, 0);
  // double punch_offset = 0.16;
  // point im = p1 + point(punch_offset, 0, 0);
  lines.push_back(linear_cut::make(p0, p1));
  lines.push_back(linear_cut::make(p1, p2));
  lines.push_back(linear_cut::make(p2, p3));
  lines.push_back(linear_cut::make(p3, p0));

  vector<hole_punch*> holes;
  vector<b_spline*> splines;
  shape_layout l(lines, holes, splines);
  gprog* p = parse_gprog(shape_layout_to_gcode_string(l, params));
  
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
