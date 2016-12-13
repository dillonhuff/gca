#pragma once

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
