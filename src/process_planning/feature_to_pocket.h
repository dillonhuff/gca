#ifndef GCA_FEATURE_TO_POCKET_H
#define GCA_FEATURE_TO_POCKET_H

#include "feature_recognition/feature_decomposition.h"
#include "geometry/homogeneous_transformation.h"
#include "process_planning/tool_access.h"
#include "backend/toolpath_generation.h"

namespace gca {

  std::vector<pocket>
  feature_pockets(feature_decomposition& r,
		  const tool_access_info& tool_info);

  std::vector<pocket>
  feature_pockets(feature_decomposition& r,
		  const point n,
		  const tool_access_info& tool_info);

  std::vector<pocket>
  feature_pockets(feature_decomposition& r,
		  const homogeneous_transform& t,
		  const tool_access_info& tool_info);

  std::vector<pocket>
  feature_pockets(const std::vector<feature*>& features,
		  const homogeneous_transform& t,
		  const tool_access_info& tool_info);
  
}

#endif
