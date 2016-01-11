#ifndef GCA_OUTPUT_H
#define GCA_OUTPUT_H

#include <vector>

#include "core/context.h"
#include "core/gprog.h"
#include "geometry/line.h"

using namespace std;

namespace gca {

  vector<cut*> lines_to_cuts(context& c, vector<line>& lines, double cutter_width);

  gprog* gcode_for_cuts(context& c, vector<cut*>& cuts);
  
  cut* sink_cut(context& c, cut* s, double l);

  void insert_sink_cuts(context& c, double l, vector<cut*>& cuts, vector<cut*>& dest);

  vector<cut*> surface_cuts(context &c,
			    point left, point right,
			    point shift, int num_cuts);

  vector<cut*> two_pass_surface(double coarse_depth, double finish_inc,
				double cutter_width,
				double x_s, double x_e, double y,
				double width);

}

#endif
