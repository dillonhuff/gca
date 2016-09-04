#include "feature_recognition/visual_debug.h"
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

  unsigned
  num_coverings_using_feature(const feature* f,
			      const std::vector<feature*>& already_covered,
			      const containment_map& cmap) {
    unsigned num_covers = 0;
    for (auto q : cmap) {
      if (!elem(q.first, already_covered) && elem(f, q.second)) {
	num_covers++;
      }
    }
    return num_covers;
  }

  std::vector<feature*>
  features_overlapped_by(const feature* f,
			 const containment_map& cmap) {
    vector<feature*> oac;
    for (auto q : cmap) {
      if (elem(f, q.second)) {
	oac.push_back(q.first);
      }
    }
    return oac;
  }

  std::vector<feature*>
  overlaps_already_covered(const feature* f,
			   const std::vector<feature*>& already_covered,
			   const containment_map& cmap) {
    vector<feature*> oac;
    for (auto q : cmap) {
      if (elem(q.first, already_covered) && elem(f, q.second)) {
	oac.push_back(q.first);
      }
    }

    cout << "# Overlaps already covered = " << oac.size() << endl;
    return oac;
  }
  
  typedef std::vector<feature*> feature_set;
  typedef std::vector<feature_set> containment_chain;

  boost::optional<feature_set>
  next_containing_set(const containment_map& cmap,
		      feature_set& last_set) {
    feature_set next_set;

    for (auto f : last_set) {
      auto l = cmap.find(f);
      if (l != end(cmap)) {

	auto containing_elems = l->second;
	if (containing_elems.size() == 0) {
	  return boost::none;
	}

	concat(next_set, containing_elems);

      } else {
	DBG_ASSERT(false);
      }
      
    }

    return sort_unique(next_set);
  }
  
  void extend_chain(const containment_map& cmap,
		    containment_chain& chain) {
    boost::optional<feature_set> containing_set =
      next_containing_set(cmap, chain.back());

    if (containing_set) {
      chain.push_back(*containing_set);
      extend_chain(cmap, chain);
    }
  }

  std::vector<containment_chain>
  build_containment_chains(const containment_map& cmap,
			   const std::vector<feature*>& contained) {
    vector<containment_chain> chains;
    for (auto f : contained) {
      containment_chain initial_chain{{f}};
      extend_chain(cmap, initial_chain);


      cout << "Chain length = " << initial_chain.size() << endl;
      
      chains.push_back(initial_chain);
    }

    return chains;
  }

  std::vector<feature*>
  pick_features_to_cut(const containment_map& cmap,
		       const std::vector<feature*>& contained) {
    vector<containment_chain> chains = build_containment_chains(cmap, contained);

    vector<feature*> to_cut;
    for (auto chain : chains) {
      cout << "CHAIN" << endl;
      for (auto fset : chain) {
      	//vtk_debug_features(fset);
      }
      concat(to_cut, chain.back());
    }

    return sort_unique(to_cut);
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


    vector<feature*> features_to_cut =
      pick_features_to_cut(cmap, contained);

    cout << "# of features to cut = " << features_to_cut.size() << endl;
    // for (auto f : features_to_cut) {
    //   vtk_debug_feature(*f);
    // }

    subtract(contained, features_to_cut);

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

  bool past_overlap(const feature& l, const feature& r) {
    point n = l.normal();
    
    pair<double, double> l_r = l.range_along(n);
    pair<double, double> r_r = r.range_along(n);

    return intervals_overlap(l_r, r_r);
  }

  std::vector<std::pair<feature*, feature*> >
  duplicate_features(feature_decomposition* target_decomp,
		     feature_decomposition* container_decomp) {
    DBG_ASSERT(angle_eps(normal(target_decomp), normal(container_decomp), 180.0, 1.0));

    vector<pair<feature*, feature*>> duplicates;
    for (auto target : collect_leaf_features(target_decomp)) {
      for (auto container : collect_leaf_features(container_decomp)) {
	if (same_base(*target, *container, 0.001) &&
	    past_overlap(*target, *container)) {
	  duplicates.push_back(make_pair(target, container));
	}
      }
    }

    return duplicates;
  }
  
  std::vector<feature_decomposition*>
  clip_top_and_bottom_features(feature_decomposition* top_decomp,
			       feature_decomposition* base_decomp) {
    vector<pair<feature*, feature*>> dup_feature_pairs =
      duplicate_features(base_decomp, top_decomp);

    vector<feature*> dup_features;
    for (auto trimmed_and_deleted_pair : dup_feature_pairs) {
      dup_features.push_back(trimmed_and_deleted_pair.second);
    }

    cout << "# of duplicate features = " << dup_features.size() << endl;
    prune_features(base_decomp, dup_features);
    prune_features(top_decomp, dup_features);

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

    return clip_top_and_bottom_features(top_decomp, base_decomp);
  }
  
}
