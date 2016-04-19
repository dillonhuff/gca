#ifndef GCA_AXIS_3_H
#define GCA_AXIS_3_H

#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"

namespace gca {

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter);

}

#endif
