#include "process_planning/tool_access.h"
#include "utils/check.h"

namespace gca {

  feature access_feature(const feature& f,
			 const tool& t,
			 const double diam,
			 const double len) {
    return f;
  }

  bool can_access_feature_with_tool(const feature& f,
				    const tool& t,
				    feature_decomposition* decomp) {
    if ((t.cut_length() < f.depth()) &&
	(t.cut_diameter() <= t.shank_diameter())) { return false; }

    feature shank_region =
      access_feature(f, t, t.shank_diameter(), t.shank_length());

    if (containing_subset(shank_region, collect_features(decomp)).size() == 0) {
      return false;
    }

    feature holder_region =
      access_feature(f, t, t.holder_diameter(), t.holder_diameter());

    if (containing_subset(holder_region, collect_features(decomp)).size() == 0) {
      return false;
    }
    
    return true;
  }
				    

  tool_access_info
  find_accessable_tools(feature_decomposition* f,
			const std::vector<tool>& tools) {
    tool_access_info info;
    for (auto ft : collect_features(f)) {
      info[ft] = {};
      for (auto t : tools) {
	if (can_access_feature_with_tool(*ft, t, f)) {
	  map_insert(info, ft, t);
	}
      }
    }
    return info;
  }

}
