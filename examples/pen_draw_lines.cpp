#include <cassert>
#include <iostream>

#include "checkers/bounds_checker.h"
#include "checkers/forbidden_tool_checker.h"
#include "synthesis/shapes_to_gcode.h"

using namespace gca;


int main() {
  arena_allocator a;
  set_system_allocator(&a);

  cut_params params;
  params.safe_height = -3.75;
  params.material_depth = 0.011;
  params.cut_depth = 0.01;
  params.push_depth = 0.005;
  params.start_loc = point(8.3, 7.6, 0);
  params.default_feedrate = 20;
  params.one_pass_only = true;
  params.pass_depth = -4.05;
  params.tools = DRILL_ONLY;

  vector<cut*> lines;
  lines.push_back(mk_linear_cut(point(8.3, 7.6, 0), point(8.5, 8.1, 0)));
  lines.push_back(mk_linear_cut(point(8.5, 8.1, 0), point(8.75, 7.2, 0)));
  vector<hole_punch*> holes;
  vector<b_spline*> splines;
  shape_layout l(lines, holes, splines);
  
  gprog* p = shape_layout_to_gcode(l, params);

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  p->print_nc_output(cout);

  // Check for errors in the output
  vector<int> permitted_tools;
  check_for_forbidden_tool_changes(permitted_tools, p);
  check_bounds(p, GCA_ABSOLUTE,
	       7, 9,
	       7, 9,
	       -4.1, -0.05);
  return 0;
}
