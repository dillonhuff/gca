#include "process_planning/feature_to_pocket.h"
#include "utils/check.h"

namespace gca {

  // TODO: Deal with the polygon holes issue
  std::vector<pocket> pockets_for_feature(const feature& f) {

    labeled_polygon_3 base = f.base();
    point n = base.normal();
    cout << "base normal = " << n << endl;
    
    DBG_ASSERT(within_eps(angle_between(n, point(0, 0, 1)), 0.0, 0.1));

    oriented_polygon ob = to_oriented_polygon(base);

    vector<oriented_polygon> holes;
    for (auto h : f.base().holes()) {
      holes.push_back(oriented_polygon(n, h));
    }

    // TODO: Add real depth values
    return {flat_pocket(1.0, 0.0, ob, holes)};
  }
  
  std::vector<pocket>
  feature_pockets_ignoring_top_face(feature_decomposition& r) {

    DBG_ASSERT(r.feature() == nullptr);
    DBG_ASSERT(r.num_children() == 1);

    auto face_feature_node = r.child(0);
    const feature* top_face_feature = face_feature_node->feature();

    vector<feature*> non_top_face_features;
    auto func = [top_face_feature, &non_top_face_features](feature* f) {
      if (f != nullptr && f != top_face_feature)
	{ non_top_face_features.push_back(f); }
    };

    traverse_bf(face_feature_node, func);

    vector<pocket> pockets;
    for (auto f : non_top_face_features) {
      concat(pockets, pockets_for_feature(*f));
    }
    
    return pockets;
  }

}
