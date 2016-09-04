#pragma once

#include <unordered_map>

#include "feature_recognition/feature_decomposition.h"
#include "synthesis/clipping_plan.h"

namespace gca {

  typedef std::unordered_map<feature*, std::vector<feature*>> containment_map;
  unsigned num_contained_features(const containment_map& m);

  containment_map
  cont_map(feature_decomposition* left,
	   feature_decomposition* right);
  
  std::vector<feature_decomposition*>
  select_top_and_bottom_features(feature_decomposition* top,
				 feature_decomposition* bottom);
  
  std::vector<feature_decomposition*>
  select_features(const triangular_mesh& part_mesh,
		  const std::vector<fixture>& fixtures);

  std::vector<feature_decomposition*>
  clip_top_and_bottom_features(feature_decomposition* top_decomp,
			       feature_decomposition* base_decomp);
  
}
