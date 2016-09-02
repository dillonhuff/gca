#include "process_planning/feature_selection.h"

namespace gca {

  std::vector<feature*>
  contained_features(feature_decomposition* to_check,
		     feature_decomposition* containing_decomp) {
    DBG_ASSERT(angle_eps(normal(to_check), normal(containing_decomp), 180.0, 1.0));

    vector<feature*> contained;
    for (auto c : collect_features(to_check)) {
      for (auto f : collect_features(containing_decomp)) {
	if (contains(*f, *c)) {
	  contained.push_back(c);
	  break;
	}
      }
    }

    return contained;
  }

  void prune_features(feature_decomposition* f,
		      const std::vector<feature*>& to_prune) {
    auto prune = [to_prune](const feature* d) { return elem(d, to_prune); };
    
    delete_leaves(f, prune);
  }
  
  std::vector<feature_decomposition*>
  select_features(const triangular_mesh& part_mesh,
		  const std::vector<fixture>& fixtures) {
    DBG_ASSERT(fixtures.size() == 2);
    auto top_fix = fixtures[0];
    auto base_fix = fixtures[1];
    
    auto top_decomp =
      build_feature_decomposition(part_mesh, top_fix.orient.top_normal());

    auto base_decomp =
      build_feature_decomposition(part_mesh, base_fix.orient.top_normal());

    vector<feature*> contained_base_features =
      contained_features(base_decomp, top_decomp);

    cout << "# of contained features = " << contained_base_features.size() << endl;
    prune_features(base_decomp, contained_base_features);
    
    return {top_decomp, base_decomp};
  }
  
}
