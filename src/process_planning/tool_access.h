#pragma once

#include <unordered_map>

#include "feature_recognition/feature_decomposition.h"
#include "synthesis/tool.h"

namespace gca {

  typedef std::unordered_map<feature*, std::vector<tool>> tool_access_info;

  tool_access_info
  find_accessable_tools(feature_decomposition* f,
			const std::vector<tool>& tools);

}
