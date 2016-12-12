#pragma once

#include "feature_recognition/chamfer_detection.h"
#include "feature_recognition/freeform_surface_detection.h"
#include "feature_recognition/feature_decomposition.h"
#include "process_planning/tool_access.h"
#include "synthesis/fixture_analysis.h"

namespace gca {

  struct direction_process_info {
    feature_decomposition* decomp;
    tool_access_info tool_info;
    std::vector<chamfer> chamfer_surfaces;
    std::vector<freeform_surface> freeform_surfaces;
  };

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools);

}
