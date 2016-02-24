#include <cassert>
#include <dirent.h>
#include <iostream>

#include "checkers/bounds_checker.h"
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
  params.default_feedrate = 20;
  params.safe_height = -3.75;
  params.one_pass_only = true;
  params.pass_depth = -4.05;
  params.start_loc = point(0, 0, 0);
  params.start_orient = point(1, 0, 0);
  params.tools = DRILL_ONLY;
  params.target_machine = PROBOTIX_V90_MK2_VFD;

  point shift(7.5, 7.5, 0);
  gprog* p = dxf_to_gcode(argv[1], params, shift);

  cout << "-- FINAL GCODE PROGRAM" << endl;
  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  p->print_nc_output(cout);

  int num_warns = check_bounds(p, GCA_ABSOLUTE,
			       1, 15,
			       1, 15,
			       -4.1, -0.05);
  cout << "Num warnings: " << num_warns << endl;
  return 0;
}
