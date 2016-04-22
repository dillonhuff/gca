#ifndef GCA_AXIS_3_H
#define GCA_AXIS_3_H

#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter);

  vector<polyline> mill_surface_lines(vector<triangle>& triangles,
				      double tool_diameter);

  vector<block> emco_f1_code(const vector<polyline>& pocket_lines);

  vector<pocket> surface_finishes(vector<triangle>& triangles);

  vector<pocket> make_pockets(vector<oriented_polygon> polygons);

  vector<oriented_polygon> preprocess_triangles(vector<triangle>& triangles);
}

#endif
