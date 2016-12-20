#pragma once

#include <unordered_map>

#include "feature_recognition/feature_decomposition.h"
#include "synthesis/tool.h"

namespace gca {

  typedef std::unordered_map<feature*, std::vector<tool>> tool_access_info;

  tool_access_info
  find_accessable_tools(feature_decomposition* f,
			const std::vector<tool>& tools);

  bool can_access_flat_feature_with_tool(const feature& f,
					 const tool& t,
					 feature_decomposition* decomp);

  std::vector<tool>
  accessable_tools_for_flat_feature(const feature& feat,
				    feature_decomposition* f,
				    const std::vector<tool>& tools);
  
}
