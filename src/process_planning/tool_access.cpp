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
      shrink_optional(f.base(), t.radius());

    if (!a_region) { return boost::none; }
    
    labeled_polygon_3 tool_region =
      dilate(shift(depth_offset*n, *a_region), diam / 2.0);

    vtk_debug_polygon(tool_region);

    cout << "region normal = " << tool_region.normal() << endl;
    check_simplicity(tool_region);
    
    return feature(len, tool_region);
  }

  bool feature_is_safe(const feature& f, feature_decomposition* decomp) {
    DBG_ASSERT(decomp != nullptr);
    
    auto features = collect_features(decomp);
    
    vector<feature*> subset = containing_subset(f, features);

    if (subset.size() > 0) { return true; }

    auto top_feature = decomp->child(0);

    point n = top_feature->feature()->normal();
    auto f_range = f.range_along(n);
    auto top_range = top_feature->feature()->range_along(n);
    
    return f_range.first >= top_range.second;
  }

  bool can_access_feature_with_tool(const feature& f,
				    const tool& t,
				    feature_decomposition* decomp) {
    // if ((t.cut_length() < f.depth()) &&
    // 	(t.cut_diameter() <= t.shank_diameter())) { return false; }

    boost::optional<feature> shank_region =
      access_feature(f, t, t.shank_diameter(), t.shank_length(), t.cut_length());

    if (!shank_region) { return false; }

    if (!feature_is_safe(*shank_region, decomp)) {
      return false;
    }

    boost::optional<feature> holder_region =
      access_feature(f, t, t.holder_diameter(), t.holder_length(), t.cut_length() + t.shank_length());

    if (!holder_region) { return false; }

    if (!feature_is_safe(*holder_region, decomp)) {
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
