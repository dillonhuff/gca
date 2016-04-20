#ifndef GCA_AXIS_3_H
#define GCA_AXIS_3_H

#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"

namespace gca {

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter);

  vector<polyline> mill_surface_lines(vector<triangle>& triangles,
				      double tool_diameter);

  vector<block> emco_f1_code(const vector<polyline>& pocket_lines);
}

#endif
