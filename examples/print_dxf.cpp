#include <cassert>
#include <dirent.h>
#include <iostream>

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
  params.safe_height = 0.35;
  params.material_depth = 0.09;
  params.cut_depth = 0.05;
  params.push_depth = 0.005;
  params.start_loc = point(0, 0, 0);
  params.start_orient = point(1, 0, 0);
  params.tools = DRILL_AND_DRAG_KNIFE;

  gprog* p = dxf_to_gcode(argv[1], params);

  cout << "-- FINAL GCODE PROGRAM" << endl;
  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  p->print_nc_output(cout);
  return 0;
}
