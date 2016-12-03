#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "process_planning/tool_access.h"
#include "utils/check.h"

namespace gca {

  std::vector<feature> build_access_features(const feature& f,
					     const tool& t,
					     const double diam,
					     const double len,
					     const double depth_offset) {
    check_simplicity(f.base());

    cout << "Checking access feature" << endl;
    //vtk_debug_feature(f);

    // boost::optional<labeled_polygon_3> a_region =
    //   shrink_optional(f.base(), t.radius());

    // if (!a_region) { return {}; }

    vector<polygon_3> a_regions = interior_offset({f.base()}, t.radius());

    cout << "Interior offset by " << t.radius() << endl;
    cout << "# of polygons = " << a_regions.size() << endl;
    //vtk_debug_polygons(a_regions);

    if (a_regions.size() == 0) { return {}; }

    for (auto& a_region : a_regions) {
      check_simplicity(a_region);
    }

    point n = f.normal();

    vector<polygon_3> tool_regions =
      exterior_offset(shift(depth_offset*n, a_regions), diam / 2.0);

    //vtk_debug_polygon(tool_region);

    for (auto& tool_region : tool_regions) {
      cout << "region normal = " << tool_region.normal() << endl;
      check_simplicity(tool_region);
    }

    // TODO: Correct this open closed issue
    vector<feature> access_features;
    for (auto tool_region : tool_regions) {
      access_features.push_back(feature(f.is_closed(), f.is_through(), len, tool_region));
    }

    return access_features;
    //    return {feature(f.is_closed(), len, tool_region)};
  }

  std::vector<feature> access_features(const feature& f,
				       feature_decomposition* decomp,
				       const tool& t,
				       const double diam,
				       const double len,
				       const double depth_offset) {
    DBG_ASSERT(decomp->num_children() == 1);
    
    feature_decomposition* top = decomp->child(0);
    vector<point> stock_ring = top->feature()->base().vertices();
    polygon_3 stock_polygon = build_clean_polygon_3(stock_ring);

    if (is_outer(f, stock_polygon)) {
      polygon_3 safe_envelope_outline =
	dilate(top->feature()->base(), 3.0);

      point pt = f.base().vertices().front();
      point n = f.normal();
      plane pl(n, pt);

      polygon_3 projected_outline = project_onto(pl, safe_envelope_outline);
      polygon_3 dummy_base =
	build_clean_polygon_3(projected_outline.vertices(), f.base().holes());

      feature open_feature(false, false, f.depth(), dummy_base);

      //vtk_debug_feature(open_feature);

      return build_access_features(open_feature, t, diam, len, depth_offset);
    }

    return build_access_features(f, t, diam, len, depth_offset);
  }

  bool feature_is_safe(const feature& f, feature_decomposition* decomp) {
    DBG_ASSERT(decomp != nullptr);

    auto top_feature = decomp->child(0);

    // TODO: Eventually make this a parameter, not just a builtin
    labeled_polygon_3 safe_envelope_outline =
      dilate(top_feature->feature()->base(), 20.0);
    labeled_polygon_3 safe_envelope =
      build_clean_polygon_3(safe_envelope_outline.vertices(), {top_feature->feature()->base().vertices()});

    point n = top_feature->feature()->normal();

    if (!(angle_eps(n, f.normal(), 0.0, 1.0))) {
      cout << "n          = " << n << endl;
      cout << "f.normal() = " << f.normal() << endl;
      DBG_ASSERT(angle_eps(n, f.normal(), 0.0, 1.0));
    }

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
    
    return false;
  }

  struct hole_properties {
    double diameter, depth;
  };

  boost::optional<hole_properties>
  through_hole_properties(const feature& f) {
    if (!f.is_closed() || !f.is_through()) { return boost::none; }
    polygon_3 base = f.base();
    
    return boost::none;
  }

  bool can_access_feature_with_tool(const feature& f,
				    const tool& t,
				    feature_decomposition* decomp) {
    if (t.type() == TWIST_DRILL) {
      boost::optional<hole_properties> h = through_hole_properties(f);
      if (h &&
	  within_eps(h->diameter, t.cut_diameter(), 0.0001) &&
	  h->depth <= t.cut_length()) {
	return true;
      }
    }

    if (t.type() != FLAT_NOSE && t.type() != BALL_NOSE) { return false; }

    auto top_feature = decomp->child(0)->feature();
    // NOTE: Top feature is always completely open
    if (&f != top_feature) {
      if ((t.cut_length() < f.depth()) &&
	  (t.cut_diameter() <= t.shank_diameter())) { return false; }
    }

    vector<feature> shank_regions = 
      access_features(f, decomp, t, t.shank_diameter(), t.shank_length(), t.cut_length());

    if (shank_regions.size() == 0) {
      cout << "Degenerate shank region!" << endl;
      return false;
    }

    for (auto shank_region : shank_regions) {
      if (!feature_is_safe(shank_region, decomp)) {
	cout << "Shank region is not safe" << endl;
	return false;
      }
    }

    vector<feature> holder_regions =
      access_features(f, decomp, t, t.holder_diameter(), t.holder_length(), t.cut_length() + t.shank_length());

    if (holder_regions.size() == 0) { return false; }

    for (auto holder_region : holder_regions) {
      if (!feature_is_safe(holder_region, decomp)) {
	cout << "Holder region is not safe" << endl;
	return false;
      }
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
