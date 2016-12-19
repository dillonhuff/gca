#include "geometry/extrusion.h"
#include "geometry/offset.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "process_planning/axis_location.h"
#include "process_planning/direction_selection.h"
#include "process_planning/feature_selection.h"
#include "synthesis/millability.h"

namespace gca {

  void clip_top_and_bottom_pairs(std::vector<direction_process_info>& dirs,
				 const std::vector<tool>& tools) {
    for (unsigned i = 0; i < dirs.size(); i++) {
      feature_decomposition* l = dirs[i].decomp;

      bool found_opposite = false;

      for (unsigned j = i + 1; j < dirs.size(); j++) {
	feature_decomposition* r = dirs[j].decomp;

	if (angle_eps(normal(l), normal(r), 180.0, 0.5)) {
	  found_opposite = true;

	  clip_leaves(l, r);
	  clip_leaves(r, l);

	  dirs[i].tool_info = find_accessable_tools(l, tools);
	  dirs[j].tool_info = find_accessable_tools(r, tools);

	  break;
	}

      }
    }

    for (unsigned i = 0; i < dirs.size(); i++) {
      auto decomp = dirs[i].decomp;
      auto& acc_info = dirs[i].tool_info;

      delete_leaves(decomp, [acc_info](feature* f) {
      	  return map_find(f, acc_info).size() == 0;
      	});

    }

    for (unsigned i = 0; i < dirs.size(); i++) {
      feature_decomposition* l = dirs[i].decomp;

      for (unsigned j = i + 1; j < dirs.size(); j++) {
	feature_decomposition* r = dirs[j].decomp;

	if (angle_eps(normal(l), normal(r), 180.0, 0.5)) {

	  clip_top_and_bottom_features(l, r);

	  break;
	}

      }
    }
    
  }

  void
  delete_inaccessable_non_leaf_nodes(feature_decomposition* decomp,
				     const tool_access_info& acc_info) {

    delete_internal_nodes(decomp, [acc_info](feature* f) {
	return map_find(f, acc_info).size() == 0;
      });
  }

  void
  remove_inaccessable_tools(freeform_surface& surf,
			    feature_decomposition* decomp) {
    // auto inds_cpy = surf.s.index_list();
    // vector<oriented_polygon> bounds = mesh_bounds(inds_cpy, surf.s.get_parent_mesh());
    // vector<vector<point> > rings;
    // for (auto& b : bounds) {
    //   rings.push_back(b.vertices());
    // }

    vector<polygon_3> polys =
      surface_boundary_polygons(surf.s.index_list(), surf.s.get_parent_mesh()); //arrange_rings(rings);

    //vtk_debug_polygons(polys);

    DBG_ASSERT(polys.size() == 1);

    polygon_3 surface_bound = polys.front();

    DBG_ASSERT(surface_bound.holes().size() == 0);

    point n = normal(decomp);
    point base_pt = min_point_in_dir(surf.s, n);
    plane base_plane(n, base_pt);

    polygon_3 base = project_onto(base_plane, surface_bound);

    double start_depth = max_in_dir(surf.s, n);
    double end_depth = min_in_dir(surf.s, n);

    double depth = start_depth - end_depth;

    DBG_ASSERT(depth >= 0.0);

    feature conservative_approximation(true, false, depth, base);

    delete_if(surf.tools,
	      [conservative_approximation, decomp](const tool& t) {
		return !can_access_flat_feature_with_tool(conservative_approximation,
							  t,
							  decomp);
	      });

  }
  
  void
  remove_inaccessable_tools(std::vector<freeform_surface>& freeform_surfs,
			    feature_decomposition* decomp) {
    for (auto& surf : freeform_surfs) {
      remove_inaccessable_tools(surf, decomp);
    }
  }

  direction_process_info
  build_direction_info(const triangular_mesh& stock,
		       const triangular_mesh& part,
		       const direction_info n,
		       const std::vector<tool>& tools) {
    feature_decomposition* decomp = build_feature_decomposition(stock, part, n.dir);
    tool_access_info info = find_accessable_tools(decomp, tools);
    vector<chamfer> chamfers = chamfer_regions(part, n.dir, tools);
    vector<freeform_surface> freeform_surfs;
    if (n.search_for_freeform_features) {
      freeform_surfs = freeform_surface_regions(part, n.dir, tools);
      remove_inaccessable_tools(freeform_surfs, decomp);
    }

    // cout << "# of freeform surfaces in " <<  n <<  " = " << freeform_surfs.size() << endl;

    return {decomp, info, chamfers, freeform_surfs};
  }
  
  std::vector<direction_process_info>
  initial_decompositions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const std::vector<tool>& tools,
			 const std::vector<direction_info>& norms) {

    vector<direction_process_info> dir_info;
    for (auto n : norms) {
      direction_process_info info =
	build_direction_info(stock, part, n, tools);
      dir_info.push_back(info);
    }

    for (auto info : dir_info) {
      feature_decomposition* decomp = info.decomp;

#ifdef VIZ_DBG
      vtk_debug_feature_decomposition(decomp);
#endif

      tool_access_info& acc_info = info.tool_info;

      delete_inaccessable_non_leaf_nodes(decomp, acc_info);

#ifdef VIZ_DBG
      vtk_debug_feature_decomposition(decomp);
#endif

    }

    clip_top_and_bottom_pairs(dir_info, tools);

    return dir_info;
  }

  struct mandatory_volume {
    triangular_mesh volume;
    point direction;
  };

  struct shared_edge {
    index_t triangle_1;
    index_t triangle_2;
    edge e;
  };

  boost::optional<shared_edge>
  common_edge(const index_t l,
	      const index_t r,
	      const triangular_mesh& part) {
    auto tl = part.triangle_vertices(l);
    auto tr = part.triangle_vertices(r);
    std::vector<index_t> shared_verts;

    for (unsigned i = 0; i < 3; i++) {
      for (unsigned j = 0; j < 3; j++) {
	if (tl.v[i] == tr.v[j]) {
	  shared_verts.push_back(tl.v[i]);
	}
      }
    }

    DBG_ASSERT(shared_verts.size() < 3);

    if (shared_verts.size() == 2) {
      return shared_edge{l, r, edge(shared_verts[0], shared_verts[1])};
    }

    return boost::none;
  }
  
  std::vector<shared_edge> all_shared_edges(const std::vector<index_t>& l_faces,
					    const std::vector<index_t>& r_faces,
					    const triangular_mesh& part) {
    vector<shared_edge> edges;
    for (auto l : l_faces) {
      for (auto r : r_faces) {
	auto ce = common_edge(l, r, part);
	if (ce) {
	  edges.push_back(*ce);
	}
      }
    }
    return edges;
  }

  index_t non_edge_vertex_1(const shared_edge e, const triangular_mesh& m) {
    triangle_t t1 = m.triangle_vertices(e.triangle_1);
    vector<index_t> all_verts{t1.v[0], t1.v[1], t1.v[2]};
    vector<index_t> edge_verts{e.e.l, e.e.r};
    subtract(all_verts, edge_verts);

    DBG_ASSERT(all_verts.size() == 1);

    return all_verts.front();
  }


  index_t non_edge_vertex_2(const shared_edge e, const triangular_mesh& m) {
    triangle_t t1 = m.triangle_vertices(e.triangle_2);
    vector<index_t> all_verts{t1.v[0], t1.v[1], t1.v[2]};
    vector<index_t> edge_verts{e.e.l, e.e.r};
    subtract(all_verts, edge_verts);

    DBG_ASSERT(all_verts.size() == 1);

    return all_verts.front();
  }
  
  bool is_valley_edge(const shared_edge e,
		      const triangular_mesh& m) {
    point na = m.face_orientation(e.triangle_1);
    point pa = m.vertex(non_edge_vertex_1(e, m));
    point pb = m.vertex(non_edge_vertex_2(e, m));

    return dot((pb - pa), na) > 0.0;
  }

  bool angle_eps(const shared_edge e,
		 const triangular_mesh& m,
		 const double angle,
		 const double tol) {
    point n1 = m.face_orientation(e.triangle_1);
    point n2 = m.face_orientation(e.triangle_2);
    return angle_eps(n1, n2, angle, tol);
  }

  bool share_orthogonal_valley_edge(const surface& l, const surface& r) {
    vector<shared_edge> shared =
      all_shared_edges(l.index_list(), r.index_list(), l.get_parent_mesh());
    for (auto s : shared) {
      if (is_valley_edge(s, l.get_parent_mesh()) &&
	  angle_eps(s, l.get_parent_mesh(), 90.0, 0.5)) {
	return true;
      }
    }

    return false;
  }

  triangular_mesh
  extrude_mandatory_volume(const surface& s,
			   const std::vector<surface>& surf_complex,
			   const point n) {
    cout << "# of surfaces = " << surf_complex.size() << endl;

    surface min_surf = min_e(surf_complex,
			     [n](const surface& sf) { return min_in_dir(sf, n); });
    surface max_surf = max_e(surf_complex,
			     [n](const surface& sf) { return max_in_dir(sf, n); });

    double dist = max_in_dir(max_surf, n) - min_in_dir(min_surf, n);

    cout << "Min distance along " << n << " = " << min_in_dir(min_surf, n) << endl;
    cout << "Max distance along " << n << " = " << max_in_dir(max_surf, n) << endl;
    cout << "Distance to extrude = " << dist << endl;

    auto vol = extrude_surface_negative(s.index_list(), s.get_parent_mesh(), n, dist);
    //vtk_debug_mesh(vol);

    return vol;
  }

  std::vector<mandatory_volume>
  build_mandatory_volumes(const std::vector<surface>& surf_complex) {
    if (surf_complex.size() == 0) { return {}; }

    std::vector<index_t> face_inds;
    for (auto& s : surf_complex) {
      concat(face_inds, s.index_list());
    }

    const triangular_mesh& m = surf_complex.front().get_parent_mesh();

    vector<mandatory_volume> vols;
    for (auto& s : surf_complex) {
      point s_n = normal(s);
      cout << "Trying normal = " << s_n << endl;

      vector<index_t> vert_or_horiz =
	select(face_inds, [s_n, m](const index_t& i) {
	    return all_parallel_to({i}, m, s_n, 1.0) ||
	    all_orthogonal_to({i}, m, s_n, 1.0);
	  });

      if (vert_or_horiz.size() == face_inds.size()) {
      
	auto millable_faces = prismatic_millable_faces(s_n, m);
	if (intersection(millable_faces, face_inds).size() == face_inds.size()) {
	  cout << "Viable direction = " << s_n << endl;

	  vols.push_back(mandatory_volume{extrude_mandatory_volume(s, surf_complex, s_n), s_n});
	}
      }
    }
    return vols;
  }

  std::vector<mandatory_volume>
  mandatory_volumes(const triangular_mesh& part) {
    auto regions = const_orientation_regions(part);
    vector<surface> const_surfs = inds_to_surfaces(regions, part);
    vector<vector<surface> > surf_complexes =
      connected_components_by_elems(const_surfs,
				    [](const surface& l, const surface& r) {
				      return share_orthogonal_valley_edge(l, r);
				    });

    delete_if(surf_complexes, [](const vector<surface>& surf_complex) {
	return surf_complex.size() < 2;
      });

    vector<mandatory_volume> volumes;
    for (auto& sc : surf_complexes) {
      //vtk_debug_highlight_inds(sc);
      concat(volumes, build_mandatory_volumes(sc));
    }

    return volumes;
  }

  std::vector<direction_process_info>
  select_mill_directions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const fixtures& f,
			 const std::vector<tool>& tools) {
    vector<direction_info> norms =
      select_cut_directions(stock, part, f, tools);

    vector<mandatory_volume> mandatory =
      mandatory_volumes(part);

    // vector<vtkSmartPointer<vtkActor> > actors{mesh_actor(part)};
    // for (auto& v : mandatory) {
    //   auto pd = polydata_for_trimesh(v.volume);
    //   color_polydata(pd, 0, 255, 0);
    //   actors.push_back(polydata_actor(pd));
    // }

    // visualize_actors(actors);

    vector<direction_process_info> dir_info =
      initial_decompositions(stock, part, tools, norms);

    return dir_info;
  }

}
