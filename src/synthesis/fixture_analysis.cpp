#include "synthesis/fixture_analysis.h"
#include "synthesis/millability.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/partition.h"
#include "utils/relation.h"

namespace gca {

  std::vector<surface>
  stable_surfaces_after_clipping(const triangular_mesh& part_mesh,
				 const triangular_mesh& aligned_workpiece_mesh) {
    auto s = outer_surfaces(part_mesh);

    vector<surface> surfs;

    auto stock_surfs = outer_surfaces(aligned_workpiece_mesh);
    assert(stock_surfs.size() == 6);
    vector<point> normals;
    for (auto s : stock_surfs) {
      index_t i = s.index_list().front();
      normals.push_back(s.face_orientation(i));
    }

    for (auto& sf : s) {
      for (auto n : normals) {
	if (within_eps(angle_between(n, sf.face_orientation(sf.front())), 0, 0.001)) {
	  surfs.push_back(sf);
	}
      }
    }
    
    return surfs;
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

    if (f.parallel_plates().size() > 0) {
      plate_height slot_height =
	*(min_element(begin(f.parallel_plates()), end(f.parallel_plates()),
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
  select_fixtures(relation<surface*, fixture*>& orient_map,
		      std::vector<unsigned>& surfaces_left,
		      const std::vector<surface*>& surfaces,
		      std::vector<fixture*>& all_orients) {
    vector<unsigned> orients_left = inds(all_orients);
    while (surfaces_left.size() > 0 && orients_left.size() > 0) {
      unsigned next_orient = orients_left.back();
      orients_left.pop_back();
      auto surfaces_cut = surfaces_millable_from(all_orients[next_orient]->orient,
						 surfaces,
						 all_orients[next_orient]->v);
      subtract(surfaces_left, surfaces_cut);
      for (auto surface_ind : surfaces_cut) {
	orient_map.insert(surface_ind, next_orient);
      }
    }
  }

  relation<surface*, fixture*>
  greedy_possible_fixtures(const std::vector<surface*>& surfaces,
			   std::vector<fixture*>& all_orients) {
    relation<surface*, fixture*> rel(surfaces, all_orients);
    vector<unsigned> surfaces_left = inds(surfaces);
    select_fixtures(rel, surfaces_left, surfaces, all_orients);
    assert(surfaces_left.size() == 0);
    return rel;
  }

  std::vector<unsigned>
  select_surfaces(unsigned orient_ind,
		  std::vector<unsigned>& surfaces_left,
		  const relation<surface*, fixture*>& possible_orientations) {
    vector<unsigned> initial_surfaces;
    for (auto i : surfaces_left) {
      if (elem(orient_ind, possible_orientations.rights_connected_to(i))) {
	initial_surfaces.push_back(i);
      }
    }
    return initial_surfaces;
  }

  unsigned
  orient_with_most_surfaces(const relation<surface*, fixture*>& possible_orientations,
			    const std::vector<unsigned>& orients_left,
			    const std::vector<unsigned>& surfaces_left) {
    unsigned max_surfs = 0;
    bool selected_some = false;
    unsigned p = 0;
    for (auto orient : orients_left) {
      vector<unsigned> s_inds = possible_orientations.lefts_connected_to(orient);
      s_inds = intersection(s_inds, surfaces_left);
      auto num_surfs_visible = s_inds.size();

      if (num_surfs_visible > max_surfs) {
    	p = orient;
    	max_surfs = num_surfs_visible;
    	selected_some = true;
      }
    }
    assert(selected_some);
    return p;
  }

  // NOTE: Assumes indexing of relation and partition are the same
  boost::optional<unsigned>
  min_cost_bucket(const surface_group& surfaces,
		  const relation<surface*, fixture*>& possible_orientations,
		  const partition<surface*, fixture*>& orients) {
    double min_cost = 10.0;
    bool found_bucket = false;
    unsigned i = 0;
    for (auto bucket_ind : orients.bucket_inds()) {
      double current_cost =
	orients.items_in_bucket_inds(bucket_ind).size() > 0 ? 0.0 : 1.0;
      bool can_contain_all =
	intersection(possible_orientations.lefts_connected_to(bucket_ind),
		     surfaces).size() == surfaces.size();
      if (can_contain_all && current_cost < min_cost) {
	found_bucket = true;
	min_cost = current_cost;
	i = bucket_ind;
      }
    }
    if (found_bucket) {
      return i;
    }
    return boost::none;
  }

  partition<surface*, fixture*>
  greedy_pick_orientations(const relation<surface*, fixture*>& possible_orientations) {
    std::vector<surface_group> surface_groups =
      convex_surface_groups(possible_orientations.left_elems());

    partition<surface*, fixture*> orients(possible_orientations.left_elems(),
					  possible_orientations.right_elems());

    for (auto surface_group : surface_groups) {
      boost::optional<unsigned> b = min_cost_bucket(surface_group,
						    possible_orientations,
						    orients);
      if (b) {
	for (auto s : surface_group) {
	  orients.assign_item_to_bucket(s, *b);
	}
      } else {
	cout << "No legal fixture for surface group" << endl;
	assert(false);
      }
    }

    return orients;
  }

  //  surface_map
  partition<surface*, fixture*>
  pick_orientations(const std::vector<surface*>& surfaces_to_cut,
		    std::vector<fixture*>& all_orients) {
    relation<surface*, fixture*> possible_orientations =
      greedy_possible_fixtures(surfaces_to_cut, all_orients);

    assert(surfaces_to_cut.size() == 0 || possible_orientations.right_size() > 0);

    auto greedy_orients =
      greedy_pick_orientations(possible_orientations);

    return greedy_orients;
  }

  fixture_setup
  make_fixture_setup(const triangular_mesh& part_mesh,
		     const fixture& f,
		     surface_list& surfaces) {
    triangular_mesh m = oriented_part_mesh(part_mesh, f.orient, f.v);
    triangular_mesh* mesh = new (allocate<triangular_mesh>()) triangular_mesh(m);
    auto pockets = make_surface_pockets(*mesh, surfaces);
    return fixture_setup(mesh, f, pockets);
  }

  std::vector<fixture_setup>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface*>& surfs_to_cut,
		      std::vector<fixture*>& all_orients) {
    partition<surface*, fixture*> surface_partition =
      pick_orientations(surfs_to_cut, all_orients);

    vector<fixture_setup> orients;
    for (auto j : surface_partition.non_empty_bucket_inds()) {
      fixture* fix = surface_partition.bucket(j);
      surface_list surfaces;
      for (auto i : surface_partition.items_in_bucket_inds(j)) {
	surfaces.push_back(surface_partition.item(i)->index_list());
      }
      orients.push_back(make_fixture_setup(part_mesh, *fix, surfaces));
    }

    return orients;
  }

  fixture_plan make_fixture_plan(const triangular_mesh& part_mesh,
				 const fixtures& f,
				 const vector<tool>& tools,
				 const workpiece w) {
    vector<surface> surfs_to_cut = surfaces_to_cut(part_mesh);

    clipping_plan wp_setups =
      workpiece_clipping_programs(w, part_mesh, surfs_to_cut, tools, f);

    vector<fixture_setup> setups = wp_setups.fixtures;
    const vector<surface>& stable_surfaces = wp_setups.stable_surfaces();

    vector<fixture> all_orients =
      all_stable_fixtures(stable_surfaces, f);

    auto orient_ptrs = ptrs(all_orients);
    auto surf_ptrs = ptrs(surfs_to_cut);
    concat(setups, orientations_to_cut(part_mesh, surf_ptrs, orient_ptrs));

    return fixture_plan(part_mesh, setups);
  }

}
