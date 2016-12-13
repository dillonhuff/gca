#pragma once

#include "feature_recognition/chamfer_detection.h"
#include "feature_recognition/freeform_surface_detection.h"
#include "feature_recognition/feature_decomposition.h"
#include "process_planning/tool_access.h"
#include "synthesis/fixture_analysis.h"

namespace gca {

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools);

}
