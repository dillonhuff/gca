#include "backend/drilled_hole_operation.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_to_pocket.h"
#include "utils/algorithm.h"
#include "utils/check.h"

namespace gca {

  std::vector<pocket> pockets_for_feature(const feature& f,
					  const std::vector<tool>& tools) {

    if (tools.size() == 0) {
      pair<double, double> feature_range = f.range_along(f.normal());
      cout << "ERROR: No available tools for feature" << endl;
      cout << "Feature depth = " << f.depth() << endl;
      cout << "Max along " << f.normal() << " = " << feature_range.second << endl;
      cout << "Min along normal " << f.normal() << " = " << feature_range.first << endl;
      vtk_debug_feature(f);
      DBG_ASSERT(tools.size() > 0);
    }

    labeled_polygon_3 base = f.base();
    point n = base.normal();

    if (!angle_eps(n, point(0, 0, 1), 0.0, 0.3)) {
      double theta = angle_between(n, point(0, 0, 1));
      cout << "base normal    = " << n << endl;
      cout << "desired normal = " << point(0, 0, 1) << endl;
      cout << "theta          = " << theta << endl;
      
      DBG_ASSERT(within_eps(theta, 0.0, 0.3));
    }

    DBG_ASSERT(f.depth() > 0.0);

    double base_z = base.vertex(0).z;
    double top_z = base_z + f.depth();

    if (!(base_z < top_z)) {
      cout << "base_z = " << base_z << endl;
      cout << "top_z  = " << top_z << endl;
      DBG_ASSERT(base_z < top_z);
    }

    point base_center = centroid(base.vertices());
    for (auto t : tools) {
      if (t.type() == TWIST_DRILL) {
	point up_direction(0, 0, 1);
	return {drilled_hole_operation(f.depth(), base_center, up_direction, t)};
      }
    }

    oriented_polygon ob = to_oriented_polygon(base);

    vector<oriented_polygon> holes;
    for (auto h : base.holes()) {
      auto hp = oriented_polygon(n, h);

      holes.push_back(hp);
    }

    if (f.is_closed()) {
      return {flat_pocket(top_z, base_z, ob, holes, tools)};
    } else {
      DBG_ASSERT(!f.is_closed());

      if (holes.size() == 0) {
	return {face_pocket(top_z, base_z, ob, tools)};
      }

      return {contour(top_z, base_z, base, tools)};
    }
  }

  std::vector<pocket>
  feature_pockets(const std::vector<feature*>& features,
		  const tool_access_info& tool_info) {

    vector<pocket> pockets;
    for (auto f : features) {
      vector<tool> tools = map_find(f, tool_info);
      concat(pockets, pockets_for_feature(*f, tools));
    }
    
    return pockets;
  }

  std::vector<pocket>
  feature_pockets(const std::vector<feature*>& features,
		  const homogeneous_transform& t,
		  const tool_access_info& tool_info) {

    vector<pocket> pockets;
    for (auto f : features) {
      vector<tool> tools = map_find(f, tool_info);
      concat(pockets, pockets_for_feature(f->apply(t), tools));
    }
    
    return pockets;

  }

  
  std::vector<pocket>
  feature_pockets(feature_decomposition& r,
		  const tool_access_info& tool_info) {

    DBG_ASSERT(r.feature() == nullptr);
    DBG_ASSERT(r.num_children() == 1);

    auto face_feature_node = r.child(0);
    vector<feature*> features = collect_features(&r);

    return feature_pockets(features, tool_info);
  }

  std::vector<pocket>
  feature_pockets(feature_decomposition& r,
		  const point n,
		  const tool_access_info& tool_info) {

    DBG_ASSERT(r.feature() == nullptr);
    DBG_ASSERT(r.num_children() == 1);

    auto face_feature_node = r.child(0);
    vector<feature*> features = collect_features(&r);

    const rotation rot = rotate_from_to(n, point(0, 0, 1));

    vector<pocket> pockets;
    for (auto f : features) {
      vector<tool> tools = map_find(f, tool_info);
      feature fr = f->apply(rot);

      DBG_ASSERT(fr.depth() > 0.0);
      concat(pockets, pockets_for_feature(fr, tools));
    }
    
    return pockets;
  }

  std::vector<pocket>
  feature_pockets(feature_decomposition& r,
		  const homogeneous_transform& t,
		  const tool_access_info& tool_info) {

    DBG_ASSERT(r.feature() == nullptr);

    if (r.num_children() == 0) { return {}; }

    DBG_ASSERT(r.num_children() == 1);

    auto face_feature_node = r.child(0);
    vector<feature*> features = collect_features(&r);

    vector<pocket> pockets;
    for (auto f : features) {
      vector<tool> tools = map_find(f, tool_info);
      concat(pockets, pockets_for_feature(f->apply(t), tools));
    }
    
    return pockets;
    
  }
  
}
