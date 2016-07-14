#include "synthesis/fixture_analysis.h"
#include "synthesis/millability.h"
#include "synthesis/workpiece_clipping.h"

namespace gca {

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const triangular_mesh& stock_mesh) {
    auto stock_surfs = outer_surfaces(stock_mesh);
    assert(stock_surfs.size() == 6);
    vector<point> normals;
    for (auto s : stock_surfs) {
      index_t i = s.index_list().front();
      normals.push_back(s.face_orientation(i));
    }

    for (auto& sf : part_surfaces) {
      for (auto n : normals) {
	if (within_eps(angle_between(n, sf.face_orientation(sf.front())), 0, 0.001)) {
	  sf.set_SA();
	}
      }
    }
  }

  // TODO: Change to actually align instead of just using displacement
  triangular_mesh align_workpiece(const std::vector<surface>& part_surfaces,
				  const workpiece& w) {
    assert(part_surfaces.size() > 0);
    const triangular_mesh& part = part_surfaces.front().get_parent_mesh();

    // TODO: Pick aligning orthogonal normals
    double part_x = diameter(point(1, 0, 0), part);
    double part_y = diameter(point(0, 1, 0), part);
    double part_z = diameter(point(0, 0, 1), part);

    double bound_x = max_in_dir(part, point(1, 0, 0));
    double bound_y = max_in_dir(part, point(0, 1, 0));
    double bound_z = max_in_dir(part, point(0, 0, 1));
    
    double w_x = w.sides[0].len();
    double w_y = w.sides[1].len();
    double w_z = w.sides[2].len();

    assert(w_x > part_x);
    assert(w_y > part_y);
    assert(w_z > part_z);

    double x_margin = (w_x - part_x) / 2.0;
    double y_margin = (w_y - part_y) / 2.0;
    double z_margin = (w_z - part_z) / 2.0;

    double x_bound = bound_x + x_margin;
    double y_bound = bound_y + y_margin;
    double z_bound = bound_z + z_margin;

    auto mesh = stock_mesh(w);
    point shift(x_bound - max_in_dir(mesh, point(1, 0, 0)),
		y_bound - max_in_dir(mesh, point(0, 1, 0)),
		z_bound - max_in_dir(mesh, point(0, 0, 1)));

    auto m =
      mesh.apply_to_vertices([shift](const point p)
			     { return p + shift; });
    return m;
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
			const std::vector<clamp_orientation>& all_orients) {
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

  fixture_setup
  make_fixture_setup(const fixture& f,
		     surface_list& surfaces) {
    triangular_mesh m = oriented_part_mesh(f.orient, f.v);
    triangular_mesh* mesh = new (allocate<triangular_mesh>()) triangular_mesh(m);
    auto pockets = make_surface_pockets(*mesh, surfaces);
    return fixture_setup(mesh, f.v, pockets);
  }

  std::vector<fixture_setup>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& surfs_to_cut,
		      std::vector<fixture>& all_orients) {
    surface_map os =
      pick_orientations(part_mesh, surfs_to_cut, all_orients);

    vector<fixture_setup> orients;
    for (auto p : os) {
      unsigned ori = p.first;
      surface_list surfaces;
      for (auto i : p.second) {
      	surfaces.push_back(surfs_to_cut[i].index_list());
      }
      orients.push_back(make_fixture_setup(all_orients[ori], surfaces));
    }
    return orients;
  }

  fixture_plan make_fixture_plan(const triangular_mesh& part_mesh,
				 const fixtures& f,
				 const vector<tool>& tools,
				 const workpiece w) {
    vector<surface> surfs_to_cut = surfaces_to_cut(part_mesh);
    pair<triangular_mesh, vector<fixture_setup>> wp_setups =
      workpiece_clipping_programs(w, part_mesh, surfs_to_cut, tools, f);

    vector<fixture_setup> setups = wp_setups.second;
    triangular_mesh& aligned_workpiece_mesh = wp_setups.first;

    auto stable_surfaces = outer_surfaces(part_mesh);
    classify_part_surfaces(stable_surfaces, aligned_workpiece_mesh);
    vector<fixture> all_orients =
      all_stable_fixtures(stable_surfaces, f);
    concat(setups, orientations_to_cut(part_mesh, surfs_to_cut, all_orients));

    return fixture_plan(part_mesh, setups);
  }

}
