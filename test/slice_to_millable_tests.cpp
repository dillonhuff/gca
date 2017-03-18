#include "catch.hpp"

#include <vtkProperty.h>

#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/surface_planning.h"
#include "process_planning/tool_access.h"
#include "system/parse_stl.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/millability.h"
#include "synthesis/visual_debug.h"

namespace gca {

  struct part_search_result {
    Nef_polyhedron part_nef;
  };

  enum decomposition_type { MILLABLE_SHAPE,
			    FILLETED_SHAPE,
			    PLANE_SLICE,
			    UNFINISHED_SHAPE };

  class part_decomposition {
  public:
    virtual decomposition_type decomp_type() const = 0;
    virtual ~part_decomposition() {}
  };

  class millable_shape : public part_decomposition {
  public:
    Nef_polyhedron shape;

    millable_shape(const Nef_polyhedron& p_shape) :
      shape(p_shape) {}

    decomposition_type decomp_type() const { return MILLABLE_SHAPE; }
  };

  class filleted : public part_decomposition {
    triangular_mesh mesh;
    std::vector<shared_edge> edges_to_fillet;

  public:
    decomposition_type decomp_type() const { return FILLETED_SHAPE; }
  };

  class plane_slice : public part_decomposition {
    Nef_polyhedron shape;
    plane slice;
    std::vector<part_decomposition*> results;

  public:
    decomposition_type decomp_type() const { return PLANE_SLICE; }

    const std::vector<part_decomposition*>& children() const { return results; }

  };
  
  class unfinished_shape : public part_decomposition {
    Nef_polyhedron shape;

  public:

    unfinished_shape(const Nef_polyhedron& p_shape) :
      shape(p_shape) {}
    
    decomposition_type decomp_type() const { return UNFINISHED_SHAPE; }

  };

  bool is_centralized(const std::vector<surface>& corner_group) {
    int num_surfaces_with_ortho_multiple_ortho_connections = 0;

    for (unsigned i = 0; i < corner_group.size(); i++) {
      int num_ortho_connections = 0;
      const surface& l = corner_group[i];
      for (unsigned j = 0; j < corner_group.size(); j++) {
	if (i != j) {
	  const surface& r = corner_group[j];
	  if (share_orthogonal_valley_edge(l, r)) {
	    num_ortho_connections++;
	  }
	}
      }

      if (num_ortho_connections > 1) {
	num_surfaces_with_ortho_multiple_ortho_connections++;
      }
    }

    return num_surfaces_with_ortho_multiple_ortho_connections <= 1;
  }

  std::vector<shared_edge>
  edges_to_fillet(const std::vector<surface>& cg,
		  const triangular_mesh& m,
		  const point dir) {
    vector<shared_edge> edges;

    for (auto& s : cg) {
      for (auto& other_s : cg ) {
	if (!s.contained_by(other_s)) {
	  auto new_edges = all_shared_edges(s.index_list(), other_s.index_list(), m);
	  delete_if(new_edges,
		    [m](const shared_edge e) {
		      return !is_valley_edge(e, m) || !angle_eps(e, m, 90, 0.5);
		    });

	  delete_if(new_edges,
		    [m, dir](const shared_edge e) {
		      point start = m.vertex(e.e.l);
		      point end = m.vertex(e.e.r);
		      point diff = end - start;
		      return !angle_eps(diff, dir, 180.0, 1.0) &&
			!angle_eps(diff, dir, 0.0, 1.0);
		    });

	  concat(edges, new_edges);
	}
      }
    }

    return edges;
    
  }

  bool all_corner_groups_millable(const std::vector<std::vector<surface> >& corner_groups) {
    for (auto& cg : corner_groups) {
      if (cg.size() > 2) {
	if (!is_centralized(cg)) {
	  return false;
	}
      }
    }
    return true;
  }

  std::vector<surface> find_access_surfaces(const std::vector<surface>& cg) {
    vector<point> norms;
    for (auto& s : cg) {
      norms.push_back(normal(s));
    }

    return find_access_surfaces(cg, norms);
  }
  
  void vtk_debug_shared_edges(const std::vector<shared_edge>& edges,
			      const triangular_mesh& m) {
    vector<polyline> lines;
    for (auto s : edges) {
      point p1 = m.vertex(s.e.l);
      point p2 = m.vertex(s.e.r);
      lines.push_back(polyline({p1, p2}));
    }

    auto lines_pd = polydata_for_polylines(lines);
    color_polydata(lines_pd, 0, 255, 0);
    auto lines_act = polydata_actor(lines_pd);
    lines_act->GetProperty()->SetLineWidth(10);

    auto mesh_pd = polydata_for_trimesh(m);
    color_polydata(mesh_pd, 255, 0, 0);
    auto mesh_act = polydata_actor(mesh_pd);
    mesh_act->GetProperty()->SetOpacity(0.4);

    visualize_actors({lines_act, mesh_act});
  }

  bool all_surfaces_are_millable_from(const point dir,
				      const std::vector<surface>& sfs) {
    if (sfs.size() == 0) { return true; }

    const triangular_mesh& m = sfs.front().get_parent_mesh();

    vector<index_t> all_inds;
    for (auto& s : sfs) {
      concat(all_inds, s.index_list());
    }
    all_inds = sort_unique(all_inds);

    auto millable_inds = millable_faces(dir, m);

    return intersection(millable_inds, all_inds).size() == all_inds.size();
  }
				      

  bool
  solveable_by_filleting(const triangular_mesh& m,
			 const std::vector<std::vector<surface> >& corner_groups) {
    vector<surface> sfs = outer_surfaces(m);
    DBG_ASSERT(sfs.size() > 0);
    vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
    vector<point> dirs;
    for (auto& p : stock_planes) {
      dirs.push_back(p.normal());
      dirs.push_back(-1*p.normal());
    }

    for (auto cg : corner_groups) {

      for (auto access_dir : dirs) {

	if (!is_centralized(cg)) {

	  if (all_surfaces_are_millable_from(access_dir, cg)) {
	    vector<shared_edge> edges = edges_to_fillet(cg, m, access_dir);
	    vtk_debug_shared_edges(edges, m);
	  }
	}

      }
    }

    return true;
  }
  
  bool is_rectilinear(const triangular_mesh& m,
		      const std::vector<std::vector<surface> >& corner_groups) {


    if (all_corner_groups_millable(corner_groups)) {
      return true;
    }

    return false;
  }

  bool simplified_corners(const std::vector<std::vector<surface> >& corner_groups,
			  const std::vector<triangular_mesh>& pos_meshes) {
    int groups_after_cut = 0;

    for (auto& m : pos_meshes) {
      auto sfc = build_surface_milling_constraints(m);
      vector<vector<surface> > mcgs =
	sfc.hard_corner_groups();

      for (auto& cg : mcgs) {
	if (!is_centralized(cg)) {
	  groups_after_cut++;
	}
      }

    }

    int groups_before = 0;
    for (auto& cg : corner_groups) {
      if (!is_centralized(cg)) {
	groups_before++;
      }
    }

    cout << "Decentralized groups before cut = " << groups_before << endl;
    cout << "Decentralized groups after  cut = " << groups_after_cut << endl;

    return groups_after_cut < groups_before;
  }


  int count_planes(const std::vector<std::vector<surface> >& corner_groups) {
    int num_planes = 0;
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	num_planes += r.size();
      }
    }

    return num_planes;
  }

  bool is_finished(part_decomposition const * const pd);
  bool not_finished(part_decomposition const * const pd) {
    return !is_finished(pd);
  }

  std::vector<part_decomposition*>
  reduce_solution(part_decomposition* pd) {
    DBG_ASSERT(pd->decomp_type() == PLANE_SLICE);

    return {};
  }

  void reduce_solutions(std::vector<part_decomposition*>& possible_solutions) {
    auto first_unfinished =
      find_if(begin(possible_solutions), end(possible_solutions), not_finished);

    if (first_unfinished == end(possible_solutions)) { return; }

    part_decomposition* to_reduce = *first_unfinished;
    remove(to_reduce, possible_solutions);

    vector<part_decomposition*> reductions =
      reduce_solution(to_reduce);
    concat(possible_solutions, reductions);

    delete to_reduce;
  }

  bool plane_slice_is_finished(plane_slice const * const pd) {

    for (auto c : pd->children()) {
      if (!is_finished(c)) {
	return false;
      }
    }

    return true;
  }

  bool is_finished(part_decomposition const * const pd) {
    switch (pd->decomp_type()) {
    case MILLABLE_SHAPE:
      return true;
    case FILLETED_SHAPE:
      return true;
    case UNFINISHED_SHAPE:
      return false;
    case PLANE_SLICE:
      return plane_slice_is_finished(static_cast<plane_slice const * const>(pd));
    default:
      DBG_ASSERT(false);
    }
  }

  void
  transfer_solved_decompositions(std::vector<part_decomposition*>& possible_solutions,
				 std::vector<part_decomposition*>& solutions) {
    auto new_solutions =
      select(possible_solutions, is_finished);
    delete_if(possible_solutions, is_finished);

    concat(solutions, new_solutions);
  }
  
  std::vector<part_decomposition*>
  search_part_space(const Nef_polyhedron& part_nef) {
    vector<triangular_mesh> ms = nef_polyhedron_to_trimeshes(part_nef);

    if (ms.size() != 1) {
      return {};
    }

    // unfinished_shape* initial_part = new unfinished_shape(part_nef);
    // vector<part_decomposition*> possible_solutions{initial_part};
    // vector<part_decomposition*> solutions;

    // while (possible_solutions.size() > 0) {
    //   reduce_solutions(possible_solutions);
    //   transfer_solved_decompositions(possible_solutions, solutions);
    // }

    // return solutions;

    auto m = ms.front();

    auto sfc = build_surface_milling_constraints(m);
    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    cout << "# of hard corner groups = " << corner_groups.size() << endl;
    //vtk_debug_mesh(m);

    // if (solveable_by_filleting(m, corner_groups)) {
    //   cout << "Solveable by filleting" << endl;
    //   return {{part_nef}};
    // }

    if (is_rectilinear(m, corner_groups)) {
      cout << "Rectilinear!" << endl;
      return {};
      //return { {{part_nef}} };
    }

    if (corner_groups.size() == 0) {
      cout << "No hard corner groups left" << endl;
      //vtk_debug_mesh(m);
      //return { {{part_nef}} };
      return {};
    }

    int num_planes = count_planes(corner_groups); //0;
    cout << "Number of possible clipping planes = " << num_planes << endl;

    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	for (auto& s : r) {
	  plane p = surface_plane(s);
	  //vtk_debug(m, p);

	  auto clipped_nef_pos = clip_nef(part_nef, p.slide(0.0001));
	  auto clipped_nef_neg = clip_nef(part_nef, p.flip().slide(0.0001));

	  auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);
	  //vtk_debug_meshes(clipped_meshes);

	  clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);
	  //vtk_debug_meshes(clipped_meshes);
	  
	  vector<triangular_mesh> pos_meshes =
	    nef_polyhedron_to_trimeshes(clipped_nef_pos);
	  concat(pos_meshes, nef_polyhedron_to_trimeshes(clipped_nef_neg));

	  if (simplified_corners(corner_groups, pos_meshes)) {

	    cout << "Simlified corners! continuing" << endl;
	  
	    auto res_pos = search_part_space(clipped_nef_pos);
	    auto res_neg = search_part_space(clipped_nef_neg);

	  }

	}
      }
    }

    return {};
  }

  double distance(const polygon_3& l, const polygon_3& r) {
    double dist = 1e25;
    for (auto lpt : l.vertices()) {
      for (auto rpt : r.vertices()) {
	double d = (lpt - rpt).len();
	if (d < dist) {
	  dist = d;
	}
      }
    }

    return dist;
  }

  bool is_deep(const feature& f, const double depth_factor) {
    double area = base_area(f);
    double area_sq = sqrt(area);
    double depth = f.depth();

    return depth > 2.0*area_sq;
  }

  bool is_deep_external(const feature& f, const double depth_factor) {
    double depth = f.depth();

    polygon_3 base = f.base();
    auto base_holes = base.holes();

    if (base_holes.size() == 0) { return false; }

    for (unsigned i = 0; i < base_holes.size(); i++) {
      polygon_3 hi = build_clean_polygon_3(base_holes[i]);
      for (unsigned j = 0; j < base_holes.size(); j++) {
	if (i != j) {
	  polygon_3 hj = build_clean_polygon_3(base_holes[j]);

	  double d = distance(hi, hj);

	  if (depth > 2.0*d) {

	    // cout << "Depth = " << depth << endl;
	    // cout << "d     = " << d << endl;
	    // vtk_debug_polygons({hi, hj});


	    return true;
	  }
	}
      }
    }

    return false;
  }
  
  std::vector<feature*> check_deep_features(const triangular_mesh& m) {
    vector<surface> sfs = outer_surfaces(m);
    DBG_ASSERT(sfs.size() > 0);

    workpiece w(10, 10, 10, ALUMINUM);
    auto stock_mesh = align_workpiece(sfs, w);

    // vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
    // vector<point> dirs;
    // for (auto& p : stock_planes) {
    //   dirs.push_back(p.normal());
    //   dirs.push_back(-1*p.normal());
    // }

    vector<surface> surfs = outer_surfaces(stock_mesh);

    DBG_ASSERT(surfs.size() == 6);

    vector<point> norms;
    for (auto ax : surfs) {
      point n = ax.face_orientation(ax.front());
      norms.push_back(n);
    }

    DBG_ASSERT(norms.size() == 6);

    vector<feature*> deep_features;
    
    for (auto& d : norms) {
      auto fd = build_min_feature_decomposition(stock_mesh, m, d);
      vector<feature*> deep_internal_features = collect_features(fd);
      delete_if(deep_internal_features,
		[](const feature* f) { return !(f->is_closed()) ||
		    !is_deep(*f, 5.0); });

      delete_if(deep_internal_features,
		[](const feature* f) {
		  auto diam = circle_diameter(f->base());
		  if (diam) { return true; }
		  return false;
		});

      //vtk_debug_features(deep_internal_features);

      concat(deep_features, deep_internal_features);

      vector<feature*> deep_external_features = collect_features(fd);
      delete_if(deep_external_features,
		[](const feature* f) { return f->is_closed() ||
		    !is_deep_external(*f, 5.0); });

      //vtk_debug_features(deep_internal_features);

      concat(deep_features, deep_external_features);

    }

    return deep_features;
  }

  struct part_split {
    Nef_polyhedron nef;
    std::vector<feature*> deep_features;
  };

  part_split build_part_split(const triangular_mesh& m) {
    auto fs = check_deep_features(m);
    return {trimesh_to_nef_polyhedron(m), fs};
  }

  part_split build_part_split(const Nef_polyhedron& m) {
    vector<feature*> fs;
    for (auto& m : nef_polyhedron_to_trimeshes(m)) {
      concat(fs, check_deep_features(m));
    }
    return {m, fs};
  }
  
  // bool no_deep_features(const std::vector<triangular_mesh>& meshes) {
  //   for (auto& m : meshes) {
  //     auto feats = check_deep_features(m);
  //     if (feats.size() > 0) {
  // 	cout << "Deep features!" << endl;
  // 	//vtk_debug_features(feats);
  // 	return false;
  //     }
  //   }
  //   return true;
  // }

  int total_deep_features(const std::vector<triangular_mesh>& meshes) {
    int total = 0;

    for (auto& m : meshes) {
      auto feats = check_deep_features(m);
      total += feats.size();
      if (feats.size() > 0) {
	cout << "Deep features!" << endl;
	//vtk_debug_features(feats);
      }
    }

    return total;
  }
  
  std::vector<std::vector<part_split> >
  split_away_deep_features(const part_split& part_nef) {

    cout << "Entering split away deep features" << endl;

    auto ms = nef_polyhedron_to_trimeshes(part_nef.nef);

    cout << "# of meshes = " << ms.size() << endl;

    if (ms.size() > 1) { return {}; }

    cout << "One mesh" << endl;

    auto m = ms.front();

    cout << "Now building surface constraints" << endl;
    
    auto sfc = build_surface_milling_constraints(m);
    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    cout << "Just before computing deep features" << endl;

    int num_deep_features = part_nef.deep_features.size(); //check_deep_features(m).size();

    cout << "# of deep features in initial part = " << num_deep_features << endl;

    // This function should not be called unless the part
    // has some deep features
    DBG_ASSERT(num_deep_features > 0);

    vector<plane> possible_slice_planes;
    
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      //      if (!is_centralized(r)) {
	for (auto& s : r) {
	  plane p = surface_plane(s);
	  possible_slice_planes.push_back(p);
	}
    }

    vector<vector<part_split> > productive_splits;
    for (auto p : possible_slice_planes) {
      auto clipped_nef_pos = clip_nef(part_nef.nef, p.slide(0.0001));
      auto clipped_nef_neg = clip_nef(part_nef.nef, p.flip().slide(0.0001));

      cout << "Clipped both" << endl;

      auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);
      //vtk_debug_meshes(clipped_meshes);

      //clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);

      //cout << "Negative meshes" << endl;
      //vtk_debug_meshes(clipped_meshes);

      vector<part_split> new_split;
      new_split.push_back(build_part_split(clipped_nef_pos));
      new_split.push_back(build_part_split(clipped_nef_neg));

      // vector<triangular_mesh> pos_meshes =
      // 	nef_polyhedron_to_trimeshes(clipped_nef_pos);
      // concat(pos_meshes, nef_polyhedron_to_trimeshes(clipped_nef_neg));

      cout << "Computing # of deep features" << endl;
      int next_deep_feats = new_split[0].deep_features.size() +
	new_split[1].deep_features.size(); //total_deep_features(pos_meshes);

      cout << "Computed # deep features = " << next_deep_feats << endl;

      if (next_deep_feats < num_deep_features) {
	cout << "Reduced the number of features, returning" << endl;
	//return {clipped_nef_pos, clipped_nef_neg};
	vtk_debug_meshes(nef_polyhedron_to_trimeshes(clipped_nef_pos));
	vtk_debug_meshes(nef_polyhedron_to_trimeshes(clipped_nef_neg));
	
	productive_splits.push_back(new_split);
      }

      cout << "Done with iteration" << endl;

    }
	//      }
    //    }

    return productive_splits;
  }

  // bool deep_features_are_solved(const Nef_polyhedron& nef) {

  //   cout << "About to convert nef" << endl;
  //   auto ms = nef_polyhedron_to_trimeshes(nef);
  //   cout << "Done converting to nef" << endl;

  //   if (ms.size() > 1) {
  //     if (total_deep_features(ms) == 0) {
  // 	return true; //solved.push_back(nef);
  //     } else {
  // 	return false; //{};
  //     }
  //   } else {

  //     auto m = ms.front();

  //     auto deep_feats = check_deep_features(m);
  //     if (deep_feats.size() == 0) {
  // 	return true; //solved.push_back(nef);
  //     } else {
  // 	return false; //parts.push_back(nef);
  //     }
  //   }
    
  // }

  bool deep_features_are_solved(const std::vector<part_split>& nefs) {
    for (auto& n : nefs) {
      //if (!deep_features_are_solved(n)) {
      if (n.deep_features.size() > 0) {
	return false;
      }
    }
    return true;
  }

  vector<vector<part_split> >
  splits_of_one_subpart(const std::vector<part_split>& nps) {
    vector<part_split> next_partial_solution = nps;
    auto f = find_if(begin(next_partial_solution),
		     end(next_partial_solution),
		     [](const part_split& n) {
		       return n.deep_features.size() > 0; //!deep_features_are_solved(n);
		     });

    DBG_ASSERT(f != end(next_partial_solution));

    cout << "TRYING TO SPLIT" << endl;
    vtk_debug_meshes(nef_polyhedron_to_trimeshes(f->nef));

    vector<vector<part_split> > next = split_away_deep_features(*f);

    // Unsplittable part
    if (next.size() == 0) { return {}; }

    next_partial_solution.erase(f);

    for (auto& n : next) {
      concat(n, next_partial_solution);
    }
    
    return next;
  }

  vector<vector<part_split> >
  solve_deep_features(const triangular_mesh& m) {

    // int num_deep_features = check_deep_features(m).size();
    // auto part_nef = trimesh_to_nef_polyhedron(m);

    // if (num_deep_features == 0) {
    //   return { {part_nef} };
    // }

    auto m_split = build_part_split(m);

    if (m_split.deep_features.size() == 0) { return {{m_split}}; }

    vector<vector<part_split> > parts{{m_split}};
    vector<vector<part_split> > solved;

    while (parts.size() > 0) {
      const auto& next_partial_solution = parts.back();

      vector<vector<part_split> > splits =
	splits_of_one_subpart(next_partial_solution);

      parts.pop_back();

      // Partial solution contained a part that could not be simplified
      if (splits.size() == 0) { return {}; }

      for (auto& split : splits) {
	if (deep_features_are_solved(split)) {
	  solved.push_back(split);
	} else {
	  parts.push_back(split);
	}
      }

    }

    return solved;
  }

  TEST_CASE("Check deep internal features") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("reversecameramount with one decomposition, multiple slices") {
      triangular_mesh m =
	parse_stl("test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

      auto res = solve_deep_features(m);

      REQUIRE(res.size() > 0);
    }

    DBG_ASSERT(false);

    SECTION("caliper with multiple decompositions") {
      triangular_mesh m =
	parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);

      auto res = solve_deep_features(m);

      REQUIRE(res.size() > 0);
    }

    SECTION("artusite no deep features") {
      triangular_mesh m =
	parse_stl("./test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);

      auto res = solve_deep_features(m);

      REQUIRE(res.size() == 1);
      REQUIRE(res.front().size() == 1);
    }

    SECTION("Rear slot, no deep features") {
      triangular_mesh m =
	parse_stl("test/stl-files/onshape_parts/Rear Slot - Rear Slot.stl", 0.0001);

      auto res = solve_deep_features(m);

      REQUIRE(res.size() == 1);
      REQUIRE(res.front().size() == 1);
    }
    
  }
  
  TEST_CASE("Parsing that weird failing print object") {
    triangular_mesh m =
      parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/CTT-CM - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);
      //parse_stl("test/stl-files/onshape_parts/Rear Slot - Rear Slot.stl", 0.0001);

      //parse_stl("test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

    auto res = search_part_space(trimesh_to_nef_polyhedron(m));

    cout << "Size of result = " << res.size() << endl;

    REQUIRE(res.size() > 0);
  }

}
