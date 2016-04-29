#ifndef MESH_TO_GCODE_H
#define MESH_TO_GCODE_H

#include "geometry/triangular_mesh.h"

namespace gca {

  typedef point workpiece_dimensions;

  enum tool_type { FLAT_NOSE, BALL_NOSE };
  enum axis { X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, C_AXIS };

  class vice {
  public:
    vice(double p_length, double p_width, double p_height, axis p_ax) {}
  };

  class tool {
  public:
    tool(double p_diameter, tool_type t) {}
  };

  class gcode_program {};

  class surface {};

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& m,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece_dimensions w_dims);
}

#endif
