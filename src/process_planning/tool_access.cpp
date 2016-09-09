#include "process_planning/tool_access.h"
#include "utils/check.h"

namespace gca {

  tool_access_info
  find_accessable_tools(feature_decomposition* f,
			const std::vector<tool>& tools) {
    tool_access_info info;
    for (auto f : collect_features(f)) {
      info[f] = {};
    }
    return info;
  }

}
