#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_to_pocket.h"
#include "utils/check.h"

namespace gca {

  // TODO: Deal with the polygon holes issue
  std::vector<pocket> pockets_for_feature(const feature& f) {

    labeled_polygon_3 base = f.base();
    point n = base.normal();
    double theta = angle_between(n, point(0, 0, 1));
    cout << "base normal = " << n << endl;
    cout << "theta       = " << theta << endl;
    
    DBG_ASSERT(within_eps(theta, 0.0, 0.3));

    //    vtk_debug_polygon(base);
    
    oriented_polygon ob = to_oriented_polygon(base);

    cout << "# of HOLES = " << base.holes().size() << endl;

    vector<oriented_polygon> holes;
    for (auto h : base.holes()) {
      auto hp = oriented_polygon(n, h);

      holes.push_back(hp);
    }

    cout << "DONE HOLES" << endl;

    double base_z = base.vertex(0).z;
    double top_z = base_z + f.depth();
    // TODO: Add real depth values
    return {flat_pocket(top_z, base_z, ob, holes)};
  }

  std::vector<pocket>
  feature_pockets(feature_decomposition& r) {

    DBG_ASSERT(r.feature() == nullptr);
    DBG_ASSERT(r.num_children() == 1);

    auto face_feature_node = r.child(0);
    vector<feature*> features = collect_features(&r);

    vector<pocket> pockets;
    for (auto f : features) {
      concat(pockets, pockets_for_feature(*f));
    }
    
    return pockets;
  }

  std::vector<pocket>
  feature_pockets(feature_decomposition& r, const point n) {

    DBG_ASSERT(r.feature() == nullptr);
    DBG_ASSERT(r.num_children() == 1);

    auto face_feature_node = r.child(0);
    vector<feature*> features = collect_features(&r);

    const rotation rot = rotate_from_to(n, point(0, 0, 1));

    vector<pocket> pockets;
    for (auto f : features) {
      concat(pockets, pockets_for_feature(f->apply(rot)));
    }
    
    return pockets;
  }

  std::vector<pocket>
  feature_pockets(feature_decomposition& r,
		  const homogeneous_transform& t,
		  const point n) {

    DBG_ASSERT(r.feature() == nullptr);
    DBG_ASSERT(r.num_children() == 1);

    auto face_feature_node = r.child(0);
    vector<feature*> features = collect_features(&r);

    vector<pocket> pockets;
    for (auto f : features) {
      concat(pockets, pockets_for_feature(f->apply(t)));
    }
    
    return pockets;
    
  }
  
}
