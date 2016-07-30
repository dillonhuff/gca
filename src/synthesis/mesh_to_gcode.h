#ifndef MESH_TO_GCODE_H
#define MESH_TO_GCODE_H

#include "geometry/surface.h"
#include "synthesis/fabrication_plan.h"
#include "synthesis/fixture_analysis.h"

namespace gca {

  ostream& operator<<(ostream& out, const workpiece& w);

  enum axis { X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, C_AXIS };

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& m,
					   const fixtures& v,
					   const vector<tool>& tools,
					   const workpiece w_dims);

  fabrication_plan make_fabrication_plan(const triangular_mesh& m,
					 const fixtures& v,
					 const vector<tool>& tools,
					 const workpiece w_dims);
  
}

#endif
