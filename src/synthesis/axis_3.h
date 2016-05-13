#ifndef GCA_AXIS_3_H
#define GCA_AXIS_3_H

#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  void select_visible_triangles(vector<triangle>& triangles);

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter,
			     double cut_depth,
			     double workpiece_height);

  vector<polyline> mill_surface_lines(vector<triangle>& triangles,
				      double tool_diameter,
				      double cut_depth,
				      double workpiece_height);

  std::vector<block> emco_f1_code(const std::vector<polyline>& pocket_lines,
				  const double safe_height);

  vector<pocket> make_pockets(vector<triangle>& triangles,
			      double workpiece_height);  

  vector<oriented_polygon> preprocess_triangles(vector<triangle>& triangles);
}

#endif
