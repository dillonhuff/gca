#include "synthesis/fixture_analysis.h"
#include "synthesis/millability.h"

namespace gca {

  bool surfaces_share_edge(const unsigned i,
			   const unsigned j,
			   const std::vector<surface>& surfaces) {
    auto ind1 = surfaces[i].index_list();
    auto ind2 = surfaces[j].index_list();
    return share_edge(ind1, ind2, surfaces[i].get_parent_mesh());
  }

  bool any_SA_surface_contains(index_t i,
			       const std::vector<surface>& surfaces) {
    for (auto surface : surfaces) {
      if (surface.is_SA() && surface.contains(i)) { return true; }
    }
    return false;
  }

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
  			  std::vector<index_t>& indices) {
    delete_if(indices,
  	      [&surfaces](index_t i)
  	      { return any_SA_surface_contains(i, surfaces); });
  }

  void remove_SA_surfaces(const std::vector<surface>& stable_surfaces,
			  std::vector<surface>& cut_surfaces) {
    delete_if(cut_surfaces,
    	      [&stable_surfaces](const surface& s) {
		for (auto other : stable_surfaces) {
		  if (s.contained_by(other)) {
		    return true;
		  }
		}
		return false;
	      });
  }

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh) {
    vector<point> normals;
    normals.push_back(point(1, 0, 0));
    normals.push_back(point(-1, 0, 0));
    normals.push_back(point(0, 1, 0));
    normals.push_back(point(0, -1, 0));
    normals.push_back(point(0, 0, 1));
    normals.push_back(point(0, 0, -1));

    for (auto& sf : part_surfaces) {
      for (auto n : normals) {
	if (within_eps(angle_between(n, sf.face_orientation(sf.front())), 0, 0.001)) {
	  sf.set_SA();
	}
      }
    }
  }

  // TODO: Change to actually align instead of just making surfaces
  // orthogonal to axes
  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w) {
    return w;
  }

  std::vector<surface> outer_surfaces(const triangular_mesh& part) {
    auto const_orient_face_indices = const_orientation_regions(part);
    vector<surface> surfaces;
    for (auto f : const_orient_face_indices) {
      assert(f.size() > 0);
      if (is_outer_surface(f, part)) {
	surfaces.push_back(surface(&part, f));
      }
    }
    return surfaces;
  }

  bool orthogonal_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 90, 0.1);
  }

  bool parallel_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 180, 0.1);
  }
  
  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces) {
    vector<stock_orientation> orients;
    for (unsigned j = 0; j < surfaces.size(); j++) {
      const surface* next_left = &(surfaces[j]);
      for (unsigned k = 0; k < surfaces.size(); k++) {
	const surface* next_right = &(surfaces[k]);
	if (parallel_flat_surfaces(next_right, next_left)) {
	  for (unsigned l = 0; l < surfaces.size(); l++) {
	    const surface* next_bottom = &(surfaces[l]);
	    if (orthogonal_flat_surfaces(next_bottom, next_left)) {
	      orients.push_back(stock_orientation(next_left,
						  next_right,
						  next_bottom));
	    }
	  }
	}
      }
    }
    assert(orients.size() > 0);
    return orients;
  }

  std::vector<unsigned>
  surfaces_millable_from(const stock_orientation& orient,
			 const std::vector<surface>& surfaces_left) {
    std::vector<index_t> millable =
      millable_faces(orient.top_normal(), orient.get_mesh());
    sort(begin(millable), end(millable));
    vector<unsigned> mill_surfaces;
    for (unsigned i = 0; i < surfaces_left.size(); i++) {
      if (surfaces_left[i].contained_by_sorted(millable)) {
	mill_surfaces.push_back(i);
      }
    }
    return mill_surfaces;
  }

  std::vector<surface>
  cut_surfaces(const triangular_mesh& part) {
    // TODO: Surface this magic number as a parameter
    double normal_degrees_delta = 30.0;
    auto inds = part.face_indexes();
    vector<vector<index_t>> delta_regions =
      normal_delta_regions(inds, part, normal_degrees_delta);
    vector<surface> surfaces;
    for (auto r : delta_regions) {
      surfaces.push_back(surface(&part, r));
    }
    return surfaces;
  }

  orientation_map
  greedy_possible_orientations(const std::vector<surface>& surfaces,
			       std::vector<stock_orientation>& all_orients) {
    // TODO: Better way to do this?
    sort(begin(all_orients), end(all_orients),
    	 [](const stock_orientation l, const stock_orientation r)
    	 { return l.top_normal().x < r.top_normal().x; });
    sort(begin(all_orients), end(all_orients),
    	 [](const stock_orientation l, const stock_orientation r)
    	 { return l.top_normal().z < r.top_normal().z; });

    orientation_map orient_map;
    vector<unsigned> surfaces_left = inds(surfaces);
    vector<unsigned> orients_left = inds(all_orients);
    while (surfaces_left.size() > 0) {
      assert(orients_left.size() > 0);
      unsigned next_orient = orients_left.back();
      orients_left.pop_back();
      auto surfaces_cut = surfaces_millable_from(all_orients[next_orient], surfaces);
      subtract(surfaces_left, surfaces_cut);
      for (auto surface_ind : surfaces_cut) {
	if (orient_map.find(surface_ind) != end(orient_map)) {
	  auto k = orient_map.find(surface_ind);
	  auto orients = k->second;
	  orients.push_back(next_orient);
	  orient_map[surface_ind] = orients;
	} else {
	  vector<unsigned> orients{next_orient};
	  orient_map[surface_ind] = orients;
	}
      }
    }
    return orient_map;
  }

  // TODO: Replace part by call to get surface underlying mesh?
  bool connected_by(const unsigned i,
  		    const unsigned j,
  		    const std::vector<surface>& surfaces,
  		    const triangular_mesh& part,
		    const unsigned orient_ind,
  		    const orientation_map& orient_map) {
    auto ind1 = surfaces[i].index_list();
    auto ind2 = surfaces[j].index_list();
    if (share_edge(ind1, ind2, part)) {
      return elem(orient_ind, orient_map.find(i)->second) &&
	elem(orient_ind, orient_map.find(j)->second);
    }
    return false;
  }

  // TODO: Is there a way to do this without const?
  std::vector<unsigned>
  select_surfaces(unsigned orient_ind,
		  std::vector<unsigned>& surfaces_left,
		  const std::vector<surface>& surfaces_to_cut,
		  const orientation_map& possible_orientations,
		  const triangular_mesh& part) {
    vector<unsigned> initial_surfaces;
    for (auto i : surfaces_left) {
      vector<unsigned> viable_orients = possible_orientations.find(i)->second;
      if (elem(orient_ind, viable_orients)) {
	initial_surfaces.push_back(i);
      }
    }
    return initial_surfaces;
  }

  surface_map
  greedy_pick_orientations(const std::vector<surface>& surfaces_to_cut,
			   const std::vector<stock_orientation>& all_orients,
			   orientation_map& possible_orientations,
			   const triangular_mesh& part) {
    vector<unsigned> surfaces_left = inds(surfaces_to_cut);
    vector<unsigned> orientations_left = inds(all_orients);
    surface_map orients;
    for (auto orient : possible_orientations) {
      if (surfaces_left.size() == 0) { return orients; }

      vector<unsigned> orients_to_choose_from = orient.second;
      auto inter_orients = intersection(orients_to_choose_from,
					orientations_left);

      if (inter_orients.size() > 0) {
	unsigned orient_ind = inter_orients.back();
	cout << "Trying orientation " << orient_ind << " with top normal ";
	cout << all_orients[orient_ind].top_normal() << endl;
	remove(orient_ind, orientations_left);
	vector<unsigned> surfaces_cut =
	  select_surfaces(orient_ind, surfaces_left, surfaces_to_cut, possible_orientations, part);
	cout << "Selected " << surfaces_cut.size() << " surfaces" << endl;
	for (auto s : surfaces_cut) {
	  cout << "\t" << s << endl;
	}
	if (surfaces_cut.size() > 0) {
	  orients[orient_ind] = surfaces_cut;
	}
	subtract(surfaces_left, surfaces_cut);
      }
    }

    assert(surfaces_left.size() == 0);
    return orients;
  }

  void print_orient_map(const orientation_map& possible_orientations,
			const std::vector<stock_orientation>& all_orients) {
    cout << "ORIENTATION MAP: " << endl;
    for (auto m : possible_orientations) {
      cout << "Surface " << m.first << " can be cut from orientations" << endl;
      for (auto orient_ind : m.second) {
    	cout << orient_ind << " with top normal " << all_orients[orient_ind].top_normal() << endl;
      }
    }
  }

  // TODO: Send to system/algorithm
  template<typename A, typename B>
  void map_insert(std::map<A, std::vector<B>>& m, A a, B b) {
    if (m.find(a) == end(m)) {
      vector<B> bs{b};
      m[a] = bs;
    } else {
      auto elems = m[a];
      elems.push_back(b);
      m[a] = elems;
    }
  }

  void
  insert_component(const unsigned orient_ind,
		   const vector<unsigned>& cc,
		   surface_map& simplified,
		   const surface_map& surface_allocations,
		   const orientation_map& possible_orients,
		   const std::vector<surface>& surfaces_to_cut) {
    bool found_better_orient = false;
    if (!found_better_orient) {
      for (auto i : cc) {
	map_insert(simplified, orient_ind, i);
      }
    }
  }
  
  surface_map
  simplify_orientations(const surface_map& surface_allocations,
			const orientation_map& possible_orients,
			const std::vector<surface>& surfaces_to_cut) {
    surface_map simplified = possible_orients;
    // for (auto p : surface_allocations) {
    //   auto orient_ind = p.first;
    //   auto ccs =
    // 	connected_components_by(p.second,
    // 				[surfaces_to_cut](const unsigned i, const unsigned j)
    // 				{
    // 				  return surfaces_share_edge(i, j, surfaces_to_cut);
    // 				});
    //   for (auto cc : ccs) {
    // 	insert_component(orient_ind,
    // 			 cc,
    // 			 simplified,
    // 			 surface_allocations,
    // 			 possible_orients,
    // 			 surfaces_to_cut);
    //   }
    // }
    assert(simplified.size() <= surface_allocations.size());
    return simplified;
  }

  surface_map
  pick_orientations(const triangular_mesh& part_mesh,
		    const std::vector<surface>& surfaces_to_cut,
		    std::vector<stock_orientation>& all_orients) {
    orientation_map possible_orientations =
      greedy_possible_orientations(surfaces_to_cut, all_orients);
    assert(surfaces_to_cut.size() == 0 || possible_orientations.size() > 0);

    auto greedy_orients =  greedy_pick_orientations(surfaces_to_cut,
						    all_orients,
						    possible_orientations,
						    part_mesh);
    return greedy_orients;
    // return simplify_orientations(greedy_orients,
    // 				 possible_orientations,
    // 				 surfaces_to_cut);
  }

  std::vector<surface> surfaces_to_cut(const triangular_mesh& part_mesh,
				       const std::vector<surface>& stable_surfaces) {
    vector<surface> surfs_to_cut = cut_surfaces(part_mesh);
    cout << "# initial faces = " << surfs_to_cut.size() << endl;
    remove_SA_surfaces(stable_surfaces, surfs_to_cut);
    cout << "# faces left = " << surfs_to_cut.size() << endl;
    return surfs_to_cut;
  }

  std::vector<std::pair<stock_orientation, surface_list>>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& stable_surfaces) {
    vector<stock_orientation> all_orients =
      all_stable_orientations(stable_surfaces);

    auto surfs_to_cut = surfaces_to_cut(part_mesh, stable_surfaces);

    surface_map os =
      pick_orientations(part_mesh, surfs_to_cut, all_orients);

    vector<pair<stock_orientation, surface_list>> orients;
    for (auto p : os) {
      unsigned ori = p.first;
      surface_list surfaces;
      for (auto i : p.second) {
	surfaces.push_back(surfs_to_cut[i].index_list());
      }
      orients.push_back(mk_pair(all_orients[ori], surfaces));
    }
    return orients;
  }

}
