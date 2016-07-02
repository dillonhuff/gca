#include "synthesis/fixture_analysis.h"
#include "synthesis/millability.h"

namespace gca {

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh) {
    // TODO: Actually compute workpiece normals
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

  // TODO: Include SB surfaces in this analysis
  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces,
			  const vice& v) {
    vector<stock_orientation> orients;
    for (unsigned j = 0; j < surfaces.size(); j++) {
      const surface* next_left = &(surfaces[j]);
      if (next_left->is_SA()) {
	for (unsigned k = 0; k < surfaces.size(); k++) {
	  const surface* next_right = &(surfaces[k]);
	  if (next_right->is_SA() &&
	      parallel_flat_surfaces(next_right, next_left)) {
	    for (unsigned l = 0; l < surfaces.size(); l++) {
	      const surface* next_bottom = &(surfaces[l]);
	      if (next_bottom->is_SA() &&
		  orthogonal_flat_surfaces(next_bottom, next_left) &&
		  orthogonal_flat_surfaces(next_bottom, next_right)) {
		orients.push_back(stock_orientation(next_left,
						    next_right,
						    next_bottom));
	      }
	    }
	  }
	}
      }
    }

    delete_if(orients,
    	      [v](const stock_orientation& orient)
    	      {
    		auto part = orient.get_left().get_parent_mesh();
    		auto left_pt = part.face_triangle(orient.get_left().front()).v1;
    		auto right_pt = part.face_triangle(orient.get_right().front()).v1;
    		return abs(signed_distance_along(left_pt - right_pt, orient.left_normal()))
    		  > v.maximum_jaw_width();
    	      });

    // TODO: Consider left and right normals as well, this filtering
    // criterion is too aggressive when all 
    sort(begin(orients), end(orients),
    	 [](const stock_orientation l, const stock_orientation r)
    	 { return l.top_normal().x < r.top_normal().x; });
    sort(begin(orients), end(orients),
    	 [](const stock_orientation l, const stock_orientation r)
    	 { return l.top_normal().y < r.top_normal().z; });
    sort(begin(orients), end(orients),
    	 [](const stock_orientation l, const stock_orientation r)
    	 { return l.top_normal().z < r.top_normal().z; });
    auto it = unique(begin(orients), end(orients),
		     [](const stock_orientation& l, const stock_orientation& r) {
		       return within_eps(l.top_normal(), r.top_normal(), 0.0001);
		     });
    orients.resize(std::distance(begin(orients), it));
    assert(orients.size() > 0);
    return orients;
  }

  std::vector<fixture>
  all_stable_fixtures(const std::vector<surface>& surfaces,
		      const fixtures& f) {
    const vice& v = f.get_vice();
    auto orients = all_stable_orientations(surfaces, v);
    vector<fixture> fixtures;

    if (f.base_plates().size() > 0) {
      plate_height slot_height =
	*(min_element(begin(f.base_plates()), end(f.base_plates()),
		      [](const plate_height l, const plate_height r)
	{ return l < r; }));
      vice vice_with_lower_slot(v, slot_height);
      for (auto orient : orients) {
	fixtures.push_back(fixture(orient, vice_with_lower_slot));
      }
    }

    for (auto orient : orients) {
      fixtures.push_back(fixture(orient, v));
    }

    return fixtures;
  }

  bool point_above_vice(const index_t i,
			const stock_orientation& orient,
			const vice& v) {
    const triangular_mesh& part = orient.get_mesh();
    point p = part.vertex(i);
    point vice_dir = orient.top_normal();
    point vice_jaw_top = orient.bottom_plane_point() + (v.jaw_height() * vice_dir);
    return signed_distance_along(p, vice_dir) >
      signed_distance_along(vice_jaw_top, vice_dir);
  }

  std::vector<index_t> vertexes_touching_fixture(const stock_orientation& orient,
						 const vice& v) {
    vector<index_t> vert_inds;
    concat(vert_inds, surface_vertexes(orient.get_left()));
    concat(vert_inds, surface_vertexes(orient.get_right()));
    if (!v.has_protective_base_plate()) {
      concat(vert_inds, surface_vertexes(orient.get_bottom()));
    }
    sort(begin(vert_inds), end(vert_inds));
    delete_if(vert_inds,
	      [v, orient](const index_t i)
	      { return point_above_vice(i, orient, v); });
    return vert_inds;
  }

  std::vector<unsigned>
  surfaces_millable_from(const stock_orientation& orient,
			 const std::vector<surface>& surfaces_left,
			 const vice& v) {
    const triangular_mesh& part = orient.get_mesh();
    std::vector<index_t> millable =
      millable_faces(orient.top_normal(), part);
    vector<index_t> verts_touching_fixture =
      vertexes_touching_fixture(orient, v);
    delete_if(millable,
	      [&verts_touching_fixture, part](const index_t face_ind)
	      { return any_vertex_in(part.triangle_vertices(face_ind),
				     verts_touching_fixture); });
    sort(begin(millable), end(millable));
    vector<unsigned> mill_surfaces;
    for (unsigned i = 0; i < surfaces_left.size(); i++) {
      if (surfaces_left[i].contained_by_sorted(millable)) {
	mill_surfaces.push_back(i);
      }
    }
    return mill_surfaces;
  }

  std::vector<surface> surfaces_to_cut(const triangular_mesh& part,
				       const std::vector<surface>& stable_surfaces) {
    vector<index_t> stable_surface_inds;
    for (auto s : stable_surfaces) {
      if (s.is_SA()) {
	concat(stable_surface_inds, s.index_list());
      }
    }
    sort(begin(stable_surface_inds), end(stable_surface_inds));

    double normal_degrees_delta = 30.0;
    auto inds = part.face_indexes();
    delete_if(inds,
	      [&stable_surface_inds](const index_t i)
	      { return binary_search(begin(stable_surface_inds),
				     end(stable_surface_inds), i); });
    
    vector<vector<index_t>> delta_regions =
      normal_delta_regions(inds, part, normal_degrees_delta);
    vector<surface> surfaces;
    for (auto r : delta_regions) {
      surfaces.push_back(surface(&part, r));
    }
    return surfaces;
    
  }

  void
  select_orientations(orientation_map& orient_map,
		      std::vector<unsigned>& surfaces_left,
		      const std::vector<surface>& surfaces,
		      std::vector<fixture>& all_orients) {
    vector<unsigned> orients_left = inds(all_orients);
    while (surfaces_left.size() > 0 && orients_left.size() > 0) {
      unsigned next_orient = orients_left.back();
      orients_left.pop_back();
      auto surfaces_cut = surfaces_millable_from(all_orients[next_orient].orient,
						 surfaces,
						 all_orients[next_orient].v);
      subtract(surfaces_left, surfaces_cut);
      for (auto surface_ind : surfaces_cut) {
	map_insert(orient_map, surface_ind, next_orient);
      }
    }
  }

  orientation_map
  greedy_possible_orientations(const std::vector<surface>& surfaces,
			       std::vector<fixture>& all_orients) {
    orientation_map orient_map;
    vector<unsigned> surfaces_left = inds(surfaces);
    select_orientations(orient_map, surfaces_left, surfaces, all_orients);
    assert(surfaces_left.size() == 0);
    return orient_map;
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

  unsigned
  orient_with_most_surfaces(const orientation_map& possible_orientations,
			    const std::vector<unsigned>& orients_left,
			    const std::vector<unsigned>& surfaces_left) {
    unsigned max_surfs = 0;
    bool selected_some = false;
    unsigned p = 0;
    for (auto orient : orients_left) {
      auto num_surfs_visible = 0;
      for (auto surf_ind : surfaces_left) {
	auto surf_orient_pair = *(possible_orientations.find(surf_ind));
	if (elem(orient, surf_orient_pair.second)) {
	  num_surfs_visible++;
	}
      }

      if (num_surfs_visible > max_surfs) {
	p = orient;
	max_surfs = num_surfs_visible;
	selected_some = true;
      }
    }
    assert(selected_some);
    return p;
  }

  surface_map
  greedy_pick_orientations(const std::vector<surface>& surfaces_to_cut,
			   const std::vector<fixture>& all_orients,
			   orientation_map& possible_orientations,
			   const triangular_mesh& part) {
    vector<unsigned> surfaces_left = inds(surfaces_to_cut);
    vector<unsigned> orientations_left = inds(all_orients);
    surface_map orients;

    while (surfaces_left.size() > 0) {

      unsigned orient_ind = orient_with_most_surfaces(possible_orientations,
						      orientations_left,
						      surfaces_left);
      cout << "Trying orientation " << orient_ind << " with top normal ";
      cout << all_orients[orient_ind].orient.top_normal() << endl;
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

  bool
  transferable(const std::vector<unsigned>& cc,
	       const std::pair<unsigned, std::vector<unsigned>>& orient_surface_inds,
	       const orientation_map& possible_orients,
	       const std::vector<surface>& surfaces_to_cut) {
    bool connected_to_any = false;
    for (auto i : cc) {
      for (auto j : orient_surface_inds.second) {
	if (surfaces_share_edge(i, j, surfaces_to_cut)) {
	  connected_to_any = true;
	}
      }
    }
    if (!connected_to_any) { return false; }
    bool can_cut_from_i = true;
    for (auto i : cc) {
      auto it = possible_orients.find(i);
      if (it != end(possible_orients)) {
	if (!elem(orient_surface_inds.first, it->second)) {
	  can_cut_from_i = false;
	  break;
	}
      } else {
	assert(false);
      }
    }
    return can_cut_from_i;
  }
  
  void
  insert_component(const unsigned orient_ind,
		   const vector<unsigned>& cc,
		   surface_map& simplified,
		   const surface_map& surface_allocations,
		   const orientation_map& possible_orients,
		   const std::vector<surface>& surfaces_to_cut) {
    bool found_better_orient = false;
    for (auto q : surface_allocations) {
      if (q.first != orient_ind && transferable(cc, q, possible_orients, surfaces_to_cut)) {
	found_better_orient = true;
	for (auto i : cc) {
	  map_insert(simplified, q.first, i);
	}
      }
    }
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
    surface_map simplified;
    for (auto p : surface_allocations) {
      auto orient_ind = p.first;
      auto ccs =
      	connected_components_by(p.second,
      				[surfaces_to_cut](const unsigned i, const unsigned j)
      				{
      				  return surfaces_share_edge(i, j, surfaces_to_cut);
      				});
      auto elems = p.second;
      for (auto cc_inds : ccs) {
	vector<unsigned> cc(cc_inds.size());
	std::transform(begin(cc_inds), end(cc_inds), begin(cc),
		       [elems](const unsigned i)
		       { return elems[i]; });

      	insert_component(orient_ind,
      			 cc,
      			 simplified,
      			 surface_allocations,
      			 possible_orients,
      			 surfaces_to_cut);
      }

    }
    assert(simplified.size() <= surface_allocations.size());
    return simplified;
  }

  surface_map
  pick_orientations(const triangular_mesh& part_mesh,
		    const std::vector<surface>& surfaces_to_cut,
		    std::vector<fixture>& all_orients) {
    orientation_map possible_orientations =
      greedy_possible_orientations(surfaces_to_cut, all_orients);
    assert(surfaces_to_cut.size() == 0 || possible_orientations.size() > 0);

    auto greedy_orients = greedy_pick_orientations(surfaces_to_cut,
						   all_orients,
						   possible_orientations,
						   part_mesh);

    return simplify_orientations(greedy_orients,
    				 possible_orientations,
    				 surfaces_to_cut);
  }

  fixture_list
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& stable_surfaces,
		      const fixtures& f) {
    vector<fixture> all_orients =
      all_stable_fixtures(stable_surfaces, f);

    auto surfs_to_cut = surfaces_to_cut(part_mesh, stable_surfaces);

    surface_map os =
      pick_orientations(part_mesh, surfs_to_cut, all_orients);

    fixture_list orients;
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

  fixture_plan make_fixture_plan(const triangular_mesh& part_mesh,
				 std::vector<surface>& part_ss,
				 const fixtures& f,
				 const vector<tool>& tools,
				 const workpiece w) {
    auto aligned_workpiece = align_workpiece(part_ss, w);
    classify_part_surfaces(part_ss, aligned_workpiece);
    auto orients = orientations_to_cut(part_mesh, part_ss, f);
    return fixture_plan(part_mesh, aligned_workpiece, orients);
  }

}
