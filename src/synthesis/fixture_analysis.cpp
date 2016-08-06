#include "synthesis/fixture_analysis.h"
#include "synthesis/millability.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/arena_allocator.h"
#include "utils/constrained_partition.h"
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
      point sf_normal = sf.face_orientation(sf.front());
      for (auto n : normals) {
	double theta = angle_between(n, sf_normal);
	if (within_eps(theta, 0, 1.0)) {
	  surfs.push_back(sf);
	}
      }
    }
    
    return surfs;
  }

  std::vector<plane> max_area_basis(const std::vector<surface>& surfaces) {
    assert(surfaces.size() > 0);
    
    vector<surface> sorted_part_surfaces = surfaces;
    sort(begin(sorted_part_surfaces), end(sorted_part_surfaces),
	 [](const surface& l, const surface& r)
	 { return l.surface_area() > r.surface_area(); });
    
    vector<surface> basis =
      take_basis(sorted_part_surfaces,
		 [](const surface& l, const surface& r)
		 { return within_eps(angle_between(normal(l), normal(r)), 90, 2.0); },
		 2);

    vector<plane> planes;
    for (auto s : basis) {
      planes.push_back(plane(normal(s), s.face_triangle(s.front()).v1));
    }

    point third_vec = cross(planes[0].normal(), planes[1].normal());
    const triangular_mesh& m = surfaces.front().get_parent_mesh();
    point third_pt = max_point_in_dir(m, third_vec);

    planes.push_back(plane(third_vec, third_pt));

    return planes;
  }

  bool right_handed(const std::vector<plane>& planes) {
    assert(planes.size() == 3);

    double d = cross(planes[0].normal(), planes[1].normal()).dot(planes[2].normal());
    return d > 0.0;
  }

  std::vector<plane> set_right_handed(const std::vector<plane>& basis) {
    if (right_handed(basis)) { return basis; }
    vector<plane> rh_basis = basis;
    plane tmp = rh_basis[0];
    rh_basis[0] = rh_basis[1];
    rh_basis[1] = tmp;
    assert(right_handed(rh_basis));
    return rh_basis;
  }

  // TODO: Change to actually align instead of just using displacement
  triangular_mesh align_workpiece(const std::vector<surface>& part_surfaces,
				  const workpiece& w) {
    assert(part_surfaces.size() > 0);
    
    const triangular_mesh& part = part_surfaces.front().get_parent_mesh();

    vector<plane> part_planes = set_right_handed(max_area_basis(part_surfaces));

    auto mesh = stock_mesh(w);
    vector<surface> sfs = outer_surfaces(mesh);
    vector<plane> basis = set_right_handed(max_area_basis(sfs));

    assert(basis.size() == 3 && basis.size() == part_planes.size());
    assert(right_handed(basis));
    assert(right_handed(part_planes));
    
    vector<plane> offset_basis;
    for (unsigned i = 0; i < basis.size(); i++) {
      double stock_diam = diameter(basis[i].normal(), mesh);
      double part_diam = diameter(part_planes[i].normal(), part);
      assert(stock_diam > part_diam);
      double margin = (stock_diam - part_diam) / 2.0;
      offset_basis.push_back(basis[i].flip().slide(margin));
    }

    auto t = mate_planes(offset_basis[0], offset_basis[1], offset_basis[2],
			 part_planes[0], part_planes[1], part_planes[2]);

    assert(t);

    return apply(*t, mesh);
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
  partition_fixture_setups(const constrained_partition<surface*, fixture*>& surface_part) {
    const triangular_mesh& part_mesh =
      surface_part.items().front()->get_parent_mesh();
    
    vector<fixture_setup> orients;
    for (auto j : surface_part.non_empty_bucket_inds()) {
      fixture* fix = surface_part.bucket(j);
      surface_list surfaces;
      for (auto i : surface_part.items_in_bucket_inds(j)) {
	surfaces.push_back(surface_part.item(i)->index_list());
      }
      orients.push_back(make_fixture_setup(part_mesh, *fix, surfaces));
    }

    return orients;
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

  constrained_partition<surface*, fixture*>
  greedy_possible_fixtures(const std::vector<surface*>& surfaces,
			   std::vector<fixture*>& all_orients) {
    relation<surface*, fixture*> rel(surfaces, all_orients);
    vector<unsigned> surfaces_left = inds(surfaces);
    select_fixtures(rel, surfaces_left, surfaces, all_orients);
    assert(surfaces_left.size() == 0);
    return constrained_partition<surface*, fixture*>(rel);
  }

  // NOTE: Assumes indexing of relation and partition are the same
  boost::optional<unsigned>
  min_cost_bucket(const surface_group& surfaces,
		  const constrained_partition<surface*, fixture*>& orients) {
    double min_cost = 10.0;
    bool found_bucket = false;
    unsigned i = 0;
    for (auto bucket_ind : orients.bucket_inds()) {
      double current_cost =
	orients.items_in_bucket_inds(bucket_ind).size() > 0 ? 0.0 : 1.0;

      if (orients.bucket_can_contain_all(bucket_ind, surfaces) &&
	  current_cost < min_cost) {
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

  void
  greedy_pick_orientations(constrained_partition<surface*, fixture*>& orients) {

    std::vector<surface_group> surface_groups =
      convex_surface_groups(orients.items());

    for (auto surface_group : surface_groups) {
      boost::optional<unsigned> b =
	min_cost_bucket(surface_group, orients);

      if (b) {
	for (auto s : surface_group) {
	  orients.assign_item_to_bucket(s, *b);
	}
      } else {
	cout << "No legal fixture for surface group" << endl;
	assert(false);
      }
    }
  }

  std::vector<fixture_setup>
  orientations_to_cut(const std::vector<surface*>& surfs_to_cut,
		      std::vector<fixture*>& all_orients) {
    if (surfs_to_cut.size() == 0) { return {}; }

    constrained_partition<surface*, fixture*> surface_part =
      greedy_possible_fixtures(surfs_to_cut, all_orients);

    assert(surface_part.all_items_are_assignable() > 0);

    greedy_pick_orientations(surface_part);

    return partition_fixture_setups(surface_part);
  }

  fixture_plan make_fixture_plan(const triangular_mesh& part_mesh,
				 const fixtures& f,
				 const vector<tool>& tools,
				 const workpiece w) {
    clipping_plan wp_setups =
      workpiece_clipping_programs(w, part_mesh, tools, f);

    vector<fixture_setup> setups = wp_setups.fixtures;
    const vector<surface>& stable_surfaces =
      wp_setups.stable_surfaces();
    auto surfs_to_cut = wp_setups.surfaces_left_to_cut();

    vector<fixture> all_orients =
      all_stable_fixtures(stable_surfaces, f);

    assert(all_orients.size() > 0 || surfs_to_cut.size() == 0);

    auto orient_ptrs = ptrs(all_orients);
    auto surf_ptrs = ptrs(surfs_to_cut);
    vector<fixture_setup> rest =
      orientations_to_cut(surf_ptrs, orient_ptrs);

    concat(setups, rest);

    return fixture_plan(part_mesh, setups, wp_setups.custom_fixtures());
  }

}
