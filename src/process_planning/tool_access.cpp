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

    //vtk_debug_polygon(tool_region);

    cout << "region normal = " << tool_region.normal() << endl;
    check_simplicity(tool_region);
    
    return feature(len, tool_region);
  }

  bool feature_is_safe(const feature& f, feature_decomposition* decomp) {
    DBG_ASSERT(decomp != nullptr);

    auto top_feature = decomp->child(0);


    // TODO: Eventually make this a parameter, not just a builtin
    labeled_polygon_3 safe_envelope = dilate(top_feature->feature()->base(), 1.0);

    point n = top_feature->feature()->normal();

    DBG_ASSERT(angle_eps(n, f.normal(), 0.0, 1.0));

    auto f_range = f.range_along(n);
    auto top_range = top_feature->feature()->range_along(n);

    if (f_range.first >= top_range.second) {
      return true;
    }
    
    auto features = collect_features(decomp);
    delete_if(features, [f_range, n](const feature* c) {
	auto c_range = c->range_along(n);
	return c_range.first >= f_range.first;
      });

    vector<labeled_polygon_3> bases{safe_envelope};
    for (auto f : features) {
      bases.push_back(f->base());
    }

    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    cout << "# bases to union = " << bases.size() << endl;

    boost_multipoly_2 result;
    result.push_back(to_boost_poly_2(apply(r, bases.front())));

    for (unsigned i = 1; i < bases.size(); i++) {
      auto& s = bases[i];

      auto bp = to_boost_poly_2(apply(r, s));
      boost_multipoly_2 r_tmp = result;
      boost::geometry::clear(result);
      boost::geometry::union_(r_tmp, bp, result);
    }

    boost_poly_2 fpoly = to_boost_poly_2(apply(r, f.base()));

    boost_multipoly_2 diff;
    boost::geometry::difference(fpoly, result, diff);

    
    if (boost::geometry::area(diff) < 0.001) { return true; }
    
    //    vector<feature*> subset = containing_subset(f, features);

    //    if (subset.size() > 0) { return true; }

    return false;
  }

  // TODO: Reintroduce commented out code when done debugging PSU mount
  bool can_access_feature_with_tool(const feature& f,
				    const tool& t,
				    feature_decomposition* decomp) {
    if ((t.cut_length() < f.depth()) &&
    	(t.cut_diameter() <= t.shank_diameter())) { return false; }

    boost::optional<feature> shank_region = 
      access_feature(f, t, t.shank_diameter(), t.shank_length(), t.cut_length());

    if (!shank_region) {
      cout << "Degenerate shank region!" << endl;
      return false;
    }

    //    auto fs = collect_features(decomp);
    //    fs.push_back(&(*shank_region));
    //vtk_debug_features(fs);

    if (!feature_is_safe(*shank_region, decomp)) {
      cout << "Shank region is not safe" << endl;
      return false;
    }

    boost::optional<feature> holder_region =
      access_feature(f, t, t.holder_diameter(), t.holder_length(), t.cut_length() + t.shank_length());

    if (!holder_region) { return false; }

    if (!feature_is_safe(*holder_region, decomp)) {
      cout << "Holder region is not safe" << endl;
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
