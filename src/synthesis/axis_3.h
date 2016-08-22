#ifndef GCA_AXIS_3_H
#define GCA_AXIS_3_H

#include "geometry/polyline.h"
#include "geometry/surface.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  // TODO: Is this needed?
  void select_visible_triangles(vector<triangle>& triangles);
  
  vector<toolpath> mill_pockets(vector<pocket>& pockets,
				const std::vector<tool>& tools,
				const material& stock_material);

  std::vector<pocket>
  make_surface_pockets(const triangular_mesh& mesh,
		       std::vector<std::vector<index_t>>& surfaces);

  std::vector<pocket> pockets_for_surfaces(const std::vector<std::vector<index_t>>& surfaces,
					   double workpiece_height,
					   const triangular_mesh& mesh);

  std::vector<pocket>
  make_surface_pockets(const triangular_mesh& part,
		       const std::vector<surface>& surfaces);

}

#endif
