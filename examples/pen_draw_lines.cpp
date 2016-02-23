#include <cassert>
#include <iostream>

#include "checkers/bounds_checker.h"
#include "checkers/forbidden_tool_checker.h"
#include "synthesis/shapes_to_gcode.h"

using namespace gca;

void letter_lines(vector<cut*>& lines,
		  char letter,
		  double x_init,
		  double y_init,
		  double letter_width,
		  double eps) {
  double xi = x_init;
  double xm = x_init + (letter_width / 2.0);
  double xl = x_init + letter_width;

  double yi = y_init;
  double ym = y_init - (letter_width / 2.0);
  double yl = y_init - letter_width;

  point p0 = point(xi, yi, 0);
  point p1 = point(xm, yi, 0);
  point p2 = point(xl, yi, 0);
  
  point p3 = point(xi, ym, 0);
  point p4 = point(xm, ym, 0);
  point p5 = point(xl, ym, 0);
  
  point p6 = point(xi, yl, 0);
  point p7 = point(xm, yl, 0);
  point p8 = point(xl, yl, 0);
  
  switch (letter) {
  case ('D'):
    lines.push_back(mk_linear_cut(p0, p6));
    lines.push_back(mk_linear_cut(p6, p5));
    lines.push_back(mk_linear_cut(p5, p0));
    break;    
  case ('E'):
    lines.push_back(mk_linear_cut(p0, p6));
    lines.push_back(mk_linear_cut(p0, p2));
    lines.push_back(mk_linear_cut(p3, p5));
    lines.push_back(mk_linear_cut(p6, p8));
    break;
  case ('H'):
    lines.push_back(mk_linear_cut(p0, p6));
    lines.push_back(mk_linear_cut(p3, p5));
    lines.push_back(mk_linear_cut(p2, p8));
    break;
  case('L'):
    lines.push_back(mk_linear_cut(p0, p6));
    lines.push_back(mk_linear_cut(p6, p8));
    break;
  case('O'):
    lines.push_back(circular_arc::make(p3, p5, p4 - p3, CLOCKWISE));
    lines.push_back(circular_arc::make(p5, p3, p4 - p5, CLOCKWISE));
    // lines.push_back(mk_linear_cut(p0, p2));
    // lines.push_back(mk_linear_cut(p2, p8));
    // lines.push_back(mk_linear_cut(p8, p6));
    // lines.push_back(mk_linear_cut(p6, p0));
    break;
  case('R'):
    lines.push_back(mk_linear_cut(p0, p6));
    lines.push_back(mk_linear_cut(p3, p8));
    lines.push_back(mk_linear_cut(p3, p5));
    lines.push_back(mk_linear_cut(p5, p2));
    lines.push_back(mk_linear_cut(p2, p0));
    break;
  case('W'):
    lines.push_back(mk_linear_cut(p0, p6));
    lines.push_back(mk_linear_cut(p6, p1));
    lines.push_back(mk_linear_cut(p1, p8));
    lines.push_back(mk_linear_cut(p8, p2));
    break;
  case('\"'):
    break;
  case(' '):
    break;
  case ('\0'):
    break;
  default:
    assert(false);
  }
}

void draw_string(double x_init,
		 double y_init,
		 const string& s,
		 vector<cut*>& lines) {
  double letter_width = 0.5;
  double eps = 0.1;
  for (unsigned i = 0; i < s.size(); i++) {
    letter_lines(lines,
		 s[i],
		 x_init + i*(eps + letter_width),
		 y_init,
		 letter_width,
		 eps);
  }
}


int main(int argc, char** argv) {
  assert(argc == 2);
  string to_draw = argv[1];
  
  arena_allocator a;
  set_system_allocator(&a);

  double x_init = 5.5;
  double y_init = 7.5;

  cut_params params;
  params.safe_height = -3.75;
  params.material_depth = 0.011;
  params.cut_depth = 0.01;
  params.push_depth = 0.005;
  params.start_loc = point(x_init, y_init, 0.0);
  params.default_feedrate = 20;
  params.one_pass_only = true;
  params.pass_depth = -4.05;
  params.target_machine = PROBOTIX_V90_MK2_VFD;
  params.tools = DRILL_ONLY;

  vector<cut*> lines;
  draw_string(x_init, y_init, to_draw, lines);
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
	       4, 12,
	       5, 10,
	       -4.1, -0.05);
  return 0;
}
