#include "process_planning/feature_selection.h"

namespace gca {

  typedef std::unordered_map<feature*, std::vector<feature*>> containment_map;

  unsigned num_contained_features(const containment_map& m) {
    unsigned num_contained = 0;
    for (auto& c : m) {
      if (c.second.size() != 0) {
	num_contained++;
      }
    }
    return num_contained;
  }

  containment_map
  cont_map(feature_decomposition* left,
	   feature_decomposition* right) {
    vector<feature*> container = collect_features(right);

    containment_map cmap;
    for (auto c : collect_features(left)) {
      cmap[c] = containing_subset(*c, container);
    }

    return cmap;
  }

  feature*
  pick_next_feature(const containment_map& cmap,
		      std::vector<feature*>& covered_features,
		      std::vector<feature*>& to_prune) {
    DBG_ASSERT(to_prune.size() > 0);
    
    auto score = [cmap, covered_features](feature* f) {
      double f_score = 0.0;

      auto l = cmap.find(f);

      if (l != end(cmap)) {
	auto cover_set = l->second;
	double num_covered =
	  static_cast<double>(intersection(cover_set, covered_features).size());
	f_score += num_covered;
      } else {
	DBG_ASSERT(false);
      }

      return f_score;
    };

    return min_e(to_prune, score);
  }
  
  // TODO: Handle cascades of coverings?
  void
  select_next_feature(const containment_map& cmap,
		      std::vector<feature*>& covered_features,
		      std::vector<feature*>& to_prune) {
    feature* next = pick_next_feature(cmap, covered_features, to_prune);

    remove(next, to_prune);
    covered_features.push_back(next);

    std::vector<feature*> newly_covered;
    for (auto c : to_prune) {
      auto l = cmap.find(c);
      
      if (l != end(cmap)) {
	auto cover_set = l->second;
	if (intersection(cover_set, covered_features).size() == cover_set.size()) {
	  cout << "Newly covered feature" << endl;
	  newly_covered.push_back(c);
	}
      } else {
	DBG_ASSERT(false);
      }
    }

    //    subtract(to_prune, newly_covered);
    concat(covered_features, newly_covered);
  }
  
  void
  cont_features(const containment_map& cmap,
		std::vector<feature*>& contained) {
    auto not_contained = [cmap](feature* f) {
      auto l = cmap.find(f);
      if (l != end(cmap)) {
	return l->second.size() == 0;
      }
		
      DBG_ASSERT(false);
    };

    unsigned original_size = contained.size();
    vector<feature*> already_picked = select(contained, not_contained);
    delete_if(contained, not_contained);

    cout << "# of features left after removing uncontained features = " << contained.size() << endl;

    while (already_picked.size() < original_size) {
      cout << "selecting another feature" << endl;
      select_next_feature(cmap, already_picked, contained);
      cout << "# of contained features left = " << contained.size() << endl;
    }

  }
  
  std::vector<feature*>
  contained_features(feature_decomposition* left,
		     feature_decomposition* right) {
    DBG_ASSERT(angle_eps(normal(left), normal(right), 180.0, 1.0));

    containment_map cmap = cont_map(left, right);
    for (auto c : cont_map(right, left)) {
      cmap[c.first] = c.second;
    }

    vector<feature*> contained = collect_features(left);
    concat(contained, collect_features(right));

    cout << "# of features                 = " << contained.size() << endl;
    cout << "total # of contained features = " << num_contained_features(cmap) << endl;

    
    cont_features(cmap, contained);

    return contained;
  }

  void prune_features(feature_decomposition* f,
		      const std::vector<feature*>& to_prune) {
    auto prune = [to_prune](const feature* d) { return elem(d, to_prune); };
    
    delete_leaves(f, prune);
  }


  std::vector<feature_decomposition*>
  select_top_and_bottom_features(feature_decomposition* top_decomp,
				 feature_decomposition* base_decomp) {
    vector<feature*> contained_base_features =
      contained_features(base_decomp, top_decomp);

    cout << "# of contained features = " << contained_base_features.size() << endl;
    prune_features(base_decomp, contained_base_features);
    prune_features(top_decomp, contained_base_features);
    
    return {top_decomp, base_decomp};
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

    return select_top_and_bottom_features(top_decomp, base_decomp);
  }
  
}
