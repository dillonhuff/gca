#include "feature_recognition/feature_decomposition.h"
#include "geometry/vtk_debug.h"
#include "process_planning/feature_to_pocket.h"
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

    DBG_ASSERT(stock_surfs.size() == 6);

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
    DBG_ASSERT(surfaces.size() > 0);
    
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
    DBG_ASSERT(planes.size() == 3);

    double d = cross(planes[0].normal(), planes[1].normal()).dot(planes[2].normal());
    return d > 0.0;
  }

  std::vector<plane> set_right_handed(const std::vector<plane>& basis) {
    if (right_handed(basis)) { return basis; }
    vector<plane> rh_basis = basis;
    plane tmp = rh_basis[0];
    rh_basis[0] = rh_basis[1];
    rh_basis[1] = tmp;
    DBG_ASSERT(right_handed(rh_basis));
    return rh_basis;
  }

  // TODO: Change to actually align instead of just using displacement
  triangular_mesh align_workpiece(const std::vector<surface>& part_surfaces,
				  const workpiece& w) {
    DBG_ASSERT(part_surfaces.size() > 0);
    
    const triangular_mesh& part = part_surfaces.front().get_parent_mesh();

    vector<plane> part_planes = set_right_handed(max_area_basis(part_surfaces));

    auto mesh = stock_mesh(w);
    vector<surface> sfs = outer_surfaces(mesh);
    vector<plane> basis = set_right_handed(max_area_basis(sfs));

    DBG_ASSERT(basis.size() == 3 && basis.size() == part_planes.size());
    DBG_ASSERT(right_handed(basis));
    DBG_ASSERT(right_handed(part_planes));
    
    vector<plane> offset_basis;
    for (unsigned i = 0; i < basis.size(); i++) {
      double stock_diam = diameter(basis[i].normal(), mesh);
      double part_diam = diameter(part_planes[i].normal(), part);
      DBG_ASSERT(stock_diam > part_diam);
      double margin = (stock_diam - part_diam) / 2.0;
      offset_basis.push_back(basis[i].flip().slide(margin));
    }

    auto t = mate_planes(offset_basis[0], offset_basis[1], offset_basis[2],
			 part_planes[0], part_planes[1], part_planes[2]);

    DBG_ASSERT(t);

    return apply(*t, mesh);
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
    if (!(surfaces_left.size() == 0)) {
      for (auto f : all_orients) {
	cout << "Considered fixture with orientation = " << f->orient.top_normal() << endl;
      }
      vector<surface> surfs;
      for (auto i : surfaces_left) {
	surface s = *(surfaces[i]);
	cout << "Uncut surface normal = " << normal(s) << endl;
      	surfs.push_back(s);
      }
      vtk_debug_highlight_inds(surfs);
      DBG_ASSERT(false);
    }
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
	DBG_ASSERT(false);
      }
    }
  }

  constrained_partition<surface*, fixture*>
  assign_surfaces_to_fixtures(const std::vector<surface*>& surfs_to_cut,
			      std::vector<fixture*>& all_orients) {
    constrained_partition<surface*, fixture*> surface_part =
      greedy_possible_fixtures(surfs_to_cut, all_orients);

    DBG_ASSERT(surface_part.all_items_are_assignable() > 0);

    greedy_pick_orientations(surface_part);

    return surface_part;
  }
  
  point normal(const fixture& f) {
    return f.orient.top_normal();
  }

  fixture
  find_by_normal(const std::vector<fixture>& fixes,
		 const point n) {
    auto r = find_if(begin(fixes), end(fixes),
		     [n](const fixture& s)
		     { return within_eps(angle_between(s.orient.top_normal(), n), 0, 1.0); });
    assert(r != end(orients));
    return *r;
  }

  std::vector<fixture_setup>
  plan_fixtures(const triangular_mesh& part_mesh,
		const std::vector<surface>& stable_surfaces,
		std::vector<surface>& surfs_to_cut,
		const fixtures& f) {

    vector<fixture> all_orients =
      all_stable_fixtures(stable_surfaces, f);

    DBG_ASSERT(all_orients.size() > 0);

    auto orient_ptrs = ptrs(all_orients);
    auto surf_ptrs = ptrs(surfs_to_cut);
    constrained_partition<surface*, fixture*> fixes =
      assign_surfaces_to_fixtures(surf_ptrs, orient_ptrs);

    vector<fixture> directions;
    for (auto i : fixes.non_empty_bucket_inds()) {
      directions.push_back(*(fixes.bucket(i)));
    }
    
    vector<feature_decomposition*> decomps;
    for (auto b : directions) {
      decomps.push_back(build_feature_decomposition(part_mesh, b.orient.top_normal()));
    }

    DBG_ASSERT(decomps.size() == directions.size());

    vector<fixture_setup> rest;
    for (unsigned i = 0; i < decomps.size(); i++) {
      fixture d = directions[i];
      feature_decomposition* decomp = decomps[i];

      auto t = mating_transform(part_mesh, d.orient, d.v);

      triangular_mesh* m =
	new (allocate<triangular_mesh>()) triangular_mesh(apply(t, part_mesh));

      vector<pocket> pockets = feature_pockets(*decomp, t, d.orient.top_normal());

      rest.push_back(fixture_setup(m, d, pockets));
    }

    return rest;
  }

  fixture_plan make_fixture_plan(const triangular_mesh& part_mesh,
				 const fixtures& f,
				 const vector<tool>& tools,
				 const std::vector<workpiece>& wps) {

    DBG_ASSERT(wps.size() > 0);

    clipping_plan wp_setups =
      workpiece_clipping_programs(wps, part_mesh, tools, f);

    vector<fixture_setup> setups = wp_setups.fixtures;

    auto num_setups = setups.size();

    DBG_ASSERT(num_setups == 6 || num_setups == 2 || num_setups == 3);

    const vector<surface>& stable_surfaces =
      wp_setups.stable_surfaces();
    auto surfs_to_cut = wp_setups.surfaces_left_to_cut();

    if (surfs_to_cut.size() > 0) {
      auto rest = plan_fixtures(part_mesh, stable_surfaces, surfs_to_cut, f);
      concat(setups, rest);
    }

    return fixture_plan(part_mesh,
			setups,
			wp_setups.custom_fixtures(),
			wp_setups.stock());
  }

}
