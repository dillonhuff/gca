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
	  vtk_debug(m, p);

	  auto clipped_nef_pos = clip_nef(part_nef, p.slide(0.0001));
	  auto clipped_nef_neg = clip_nef(part_nef, p.flip().slide(0.0001));

	  auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);
	  vtk_debug_meshes(clipped_meshes);

	  clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);
	  vtk_debug_meshes(clipped_meshes);
	  
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

  bool is_deep(const feature& f, const double depth_factor) {
    double area = base_area(f);
    double area_sq = sqrt(area);
    double depth = f.depth();

    return depth > 2.0*area_sq;
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
      auto fd = build_feature_decomposition(stock_mesh, m, d);
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
    }

    return deep_features;
  }

  bool no_deep_features(const std::vector<triangular_mesh>& meshes) {
    for (auto& m : meshes) {
      auto feats = check_deep_features(m);
      if (feats.size() > 0) {
	cout << "Deep features!" << endl;
	vtk_debug_features(feats);
	return false;
      }
    }
    return true;
  }

  bool solve_deep_features(const triangular_mesh& m) {

    auto sfc = build_surface_milling_constraints(m);
    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    auto part_nef = trimesh_to_nef_polyhedron(m);
    
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	for (auto& s : r) {
	  plane p = surface_plane(s);
	  vtk_debug(m, p);

	  auto clipped_nef_pos = clip_nef(part_nef, p.slide(0.0001));
	  auto clipped_nef_neg = clip_nef(part_nef, p.flip().slide(0.0001));

	  auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);
	  vtk_debug_meshes(clipped_meshes);

	  clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);
	  vtk_debug_meshes(clipped_meshes);

	  vector<triangular_mesh> pos_meshes =
	    nef_polyhedron_to_trimeshes(clipped_nef_pos);
	  concat(pos_meshes, nef_polyhedron_to_trimeshes(clipped_nef_neg));

	  if (no_deep_features(pos_meshes)) {
	    cout << "Simlified corners! continuing" << endl;
	    vtk_debug_meshes(pos_meshes);
	    return true;
	  }

	}
      }
    }

    return false;
    
  }

  TEST_CASE("Check deep internal features") {
    arena_allocator a;
    set_system_allocator(&a);

    triangular_mesh m =
      //parse_stl("./test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/CTT-CM - Part 1.stl", 0.0001);
      //parse_stl("./test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);
      //parse_stl("test/stl-files/onshape_parts/Rear Slot - Rear Slot.stl", 0.0001);

      parse_stl("test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

    //check_deep_features(m);

    bool res = solve_deep_features(m);

    REQUIRE(res);
    
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
