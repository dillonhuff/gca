#pragma once

#include "feature_recognition/chamfer_detection.h"
#include "feature_recognition/freeform_surface_detection.h"
#include "feature_recognition/feature_decomposition.h"
#include "process_planning/axis_location.h"
#include "process_planning/tool_access.h"
#include "synthesis/fixture_analysis.h"

namespace gca {

  struct direction_process_info {
    feature_decomposition* decomp;
    tool_access_info tool_info;
    std::vector<chamfer> chamfer_surfaces;
    std::vector<freeform_surface> freeform_surfaces;
  };

  std::vector<direction_process_info>
  select_mill_directions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const fixtures& f,
			 const std::vector<tool>& tools);

  boost::optional<direction_process_info>
  find_outer_curve(std::vector<direction_process_info>& dir_info);

  direction_process_info
  build_direction_info(const triangular_mesh& stock,
		       const triangular_mesh& part,
		       const std::vector<triangular_mesh>& mandatory_meshes,
		       const direction_info n,
		       const std::vector<tool>& tools);
  
}
