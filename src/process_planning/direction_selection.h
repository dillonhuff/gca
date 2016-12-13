#pragma once

#include "feature_recognition/chamfer_detection.h"
#include "feature_recognition/freeform_surface_detection.h"
#include "feature_recognition/feature_decomposition.h"
#include "process_planning/tool_access.h"

namespace gca {

  struct direction_process_info {
    feature_decomposition* decomp;
    tool_access_info tool_info;
    std::vector<chamfer> chamfer_surfaces;
    std::vector<freeform_surface> freeform_surfaces;
  };

  std::vector<direction_process_info>
  initial_decompositions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const std::vector<tool>& tools,
			 const std::vector<point>& norms);
  
}
