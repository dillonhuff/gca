#ifndef GCA_OUTPUT_H
#define GCA_OUTPUT_H

#include <vector>

#include "core/context.h"
#include "core/gprog.h"
#include "geometry/line.h"
#include "synthesis/linear_cut.h"
#include "synthesis/machine.h"

using namespace std;

namespace gca {

  vector<linear_cut*> lines_to_cuts(vector<line>& lines, double cutter_width);

  gprog* gcode_for_cuts(vector<linear_cut*>& cuts);
  
  linear_cut* sink_cut(linear_cut* s, double l);

  void insert_sink_cuts(double l, vector<linear_cut*>& cuts, vector<linear_cut*>& dest);

  void from_to_with_G0(gprog* p, point from, point to);

  vector<linear_cut*> surface_cuts(point left, point right,
				   point shift, int num_cuts);

  vector<linear_cut*> two_pass_surface(double coarse_depth, double finish_inc,
				       double cutter_width,
				       double x_s, double x_e, double y,
				       double width);

  void from_to_with_G0(gprog* p, point from, point to);
  
  gprog* initial_gprog(machine_name m);
  gprog* append_footer(gprog* p, machine_name m);
  void append_drill_header(gprog* p, machine_name m);
  void append_drag_knife_transfer(gprog* p, machine_name m);

  vector<cut*> from_to_with_G0_height(point current_loc,
				      point next_loc,
				      double safe_height,
				      value* feedrate);
}

#endif
