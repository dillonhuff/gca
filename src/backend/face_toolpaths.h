#pragma once

#include "backend/toolpath.h"
#include "geometry/polygon_3.h"

namespace gca {

  struct face_parameters {
    double depth_of_cut;
    double width_of_cut;
    double feedrate;
    double spindle_speed;
  };

  toolpath rough_face(const face_parameters& f,
		      const double safe_z,
		      const double start_depth,
		      const double end_depth,
		      const polygon_3& face,
		      const tool& t);

}
