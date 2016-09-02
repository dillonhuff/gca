#pragma once

#include "feature_recognition/feature_decomposition.h"
#include "synthesis/clipping_plan.h"

namespace gca {

  std::vector<feature_decomposition*>
  select_features(const triangular_mesh& part_mesh,
		  const std::vector<fixture>& fixtures);

}
