#ifndef GCA_AXIS_3_H
#define GCA_AXIS_3_H

#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  // TODO: Is this needed?
  void select_visible_triangles(vector<triangle>& triangles);
  
  std::vector<pocket> make_pockets(const triangular_mesh& mesh,
				   const double workpiece_height);

  std::vector<std::vector<index_t>> make_surfaces(const triangular_mesh& mesh);

  vector<toolpath> mill_pockets(vector<pocket>& pockets,
				const std::vector<tool>& tools,
				const material& stock_material);

  std::vector<pocket>
  make_surface_pockets(const triangular_mesh& mesh,
		       std::vector<std::vector<index_t>>& surfaces);
  
}

#endif
