#include "feature_recognition/visual_debug.h"
#include "process_planning/tool_access.h"
#include "utils/check.h"

namespace gca {

  boost::optional<feature> access_feature(const feature& f,
					  const tool& t,
					  const double diam,
					  const double len,
					  const double depth_offset) {
    point n = f.normal();
    boost::optional<labeled_polygon_3> a_region =
      shrink_optional(f.base(), t.diameter());

    if (!a_region) { return boost::none; }
    
    labeled_polygon_3 tool_region = dilate(shift(depth_offset*n, *a_region), diam);

    vtk_debug_polygon(tool_region);

    cout << "region normal = " << tool_region.normal() << endl;
    check_simplicity(tool_region);
    
    
    return feature(len, tool_region);
  }

  bool can_access_feature_with_tool(const feature& f,
				    const tool& t,
				    feature_decomposition* decomp) {
    // if ((t.cut_length() < f.depth()) &&
    // 	(t.cut_diameter() <= t.shank_diameter())) { return false; }

    boost::optional<feature> shank_region =
      access_feature(f, t, t.shank_diameter(), t.shank_length(), t.cut_length());

    if (!shank_region) { return false; }

    if (containing_subset(*shank_region, collect_features(decomp)).size() == 0) {
      return false;
    }

    boost::optional<feature> holder_region =
      access_feature(f, t, t.holder_diameter(), t.holder_length(), t.cut_length() + t.shank_length());

    if (!holder_region) { return false; }

    if (containing_subset(*holder_region, collect_features(decomp)).size() == 0) {
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
