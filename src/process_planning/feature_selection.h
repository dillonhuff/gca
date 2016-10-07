#pragma once

#include <unordered_map>

#include "feature_recognition/feature_decomposition.h"
#include "process_planning/tool_access.h"
#include "synthesis/clipping_plan.h"

namespace gca {

  typedef std::unordered_map<feature*, std::vector<feature*>> containment_map;

  struct feature_selection {
    std::vector<feature_decomposition*> decomps;
    std::vector<tool_access_info> access_info;
  };

  unsigned num_contained_features(const containment_map& m);

  containment_map
  cont_map(feature_decomposition* left,
	   feature_decomposition* right);
  
  feature_selection
  select_features(const triangular_mesh& stock,
		  const triangular_mesh& part_mesh,
		  const std::vector<fixture>& fixtures,
		  const std::vector<tool>& tools);

  std::vector<feature_decomposition*>
  clip_top_and_bottom_features(feature_decomposition* top_decomp,
			       feature_decomposition* base_decomp);

  std::vector<feature*>
  unreachable_features(const std::vector<feature*> features,
		       const fixture& fix);

  void clip_leaves(feature_decomposition* top_decomp,
		   feature_decomposition* base_decomp);
  
}
