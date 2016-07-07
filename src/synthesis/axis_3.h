#ifndef GCA_AXIS_3_H
#define GCA_AXIS_3_H

#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  void select_visible_triangles(vector<triangle>& triangles);
  
  std::vector<block> emco_f1_code(const std::vector<polyline>& pocket_lines,
				  const double safe_height);

  std::vector<pocket> make_pockets(const triangular_mesh& mesh,
				   const double workpiece_height);

  std::vector<pocket>
  make_surface_pockets(const std::vector<std::vector<index_t>>& sfs,
		       const triangular_mesh& mesh,
		       double workpiece_height);
  
  std::vector<std::vector<index_t>> make_surfaces(const triangular_mesh& mesh);

  vector<polyline> mill_pockets(vector<pocket>& pockets,
				const tool& t,
				double cut_depth);

}

#endif
