#include "geometry/extrusion.h"
#include "geometry/offset.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "feature_recognition/prismatic_feature_utils.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/axis_location.h"
#include "process_planning/direction_selection.h"
#include "process_planning/feature_selection.h"
#include "process_planning/mandatory_volumes.h"


namespace gca {

  void check_feature_depths(const std::vector<direction_process_info>& dir_info) {
    for (auto& d : dir_info) {
      for (auto f : collect_features(d.decomp)) {
	if (!(f->depth() > 0.0)) {
	  cout << "Feature depth       = " << f->depth() << endl;
	  cout << "Feature normal      = " << f->normal() << endl;
	  cout << "Feature is closed   = " << f->is_closed() << endl;
	  cout << "Feature is through  = " << f->is_through() << endl;
	  
	  vtk_debug_feature(*f);
	  DBG_ASSERT(f->depth() > 0.0);
	}
      }
    }
  }

  template<typename A, typename B>
  B map_contains(const A& a, const std::unordered_map<A, B>& m) {
    auto f = m.find(a);
    if (f == std::end(m)) {
      return false;
    }

    return true;
  }

  template<typename A, typename B>
  B map_find(const A& a, const std::map<A, B>& m) {
    auto f = m.find(a);
    if (f == std::end(m)) {
      return false;
    }

    return true;
  }
  
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

	  for (feature* f : collect_features(dirs[i].decomp)) {
	    if (!map_contains(f, dirs[i].tool_info)) {
	      auto usable_tools =
		accessable_tools_for_flat_features(*f, dirs[i].decomp, tools);
	      map_insert(dirs[i].tool_info, f, usable_tools);
	    }
	  }

	  for (feature* f : collect_features(dirs[j].decomp)) {
	    if (!map_contains(f, dirs[j].tool_info)) {
	      auto usable_tools =
		accessable_tools_for_flat_features(*f, dirs[j].decomp, tools);
	      map_insert(dirs[j].tool_info, f, usable_tools);
	    }
	  }
	  
	  //dirs[i].tool_info = find_accessable_tools(l, tools);
	  //dirs[j].tool_info = find_accessable_tools(r, tools);

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

    // for (auto& d : dirs) {
    //   delete_leaves(d.decomp, [](feature* f) {
    // 	  return within_eps(f->depth(), 0.0);
    // 	});
    // }
    
    
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
		       const std::vector<triangular_mesh>& mandatory_meshes,
		       const direction_info n,
		       const std::vector<tool>& tools) {
    vector<triangular_mesh> meshes = mandatory_meshes;
    meshes.push_back(part);

    feature_decomposition* decomp = build_feature_decomposition(stock, meshes, n.dir);
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
    // auto mvs = mandatory_volumes(part);
    // vector<vtkSmartPointer<vtkActor> > ptrs{polydata_actor(polydata_for_trimesh(part))};
    // for (auto& mv : mvs) {
    //   for (auto& m : mv) {
    // 	auto pd = polydata_for_trimesh(m.volume);
    // 	color_polydata(pd, 0, 255, 0);
    // 	ptrs.push_back(polydata_actor(pd));
    //   }
    // }

    // visualize_actors(ptrs);
    
    
    vector<direction_process_info> dir_info;
    for (auto n : norms) {

      vector<triangular_mesh> meshes{};

      // for (auto& mv : mvs) {
      // 	bool is_mandatory_direction =
      // 	  any_of(begin(mv), end(mv),
      // 		 [n](const mandatory_volume& mv) {
      // 		   return angle_eps(mv.direction, n.dir, 0.0, 0.5);
      // 		 });

      // 	if (!is_mandatory_direction) {
      // 	  for (auto& m : mv) {
      // 	    meshes.push_back(m.volume);
      // 	  }
      // 	}
      // }
      
      // for (auto& md : mvs) {
      // 	for (auto& m : md) {
      // 	  meshes.push_back(m.volume);
      // 	}
      // }
      
      direction_process_info info =
	build_direction_info(stock, part, meshes, n, tools);
      dir_info.push_back(info);
    }

    check_feature_depths(dir_info);

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

  std::vector<direction_process_info>
  select_mill_directions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const fixtures& f,
			 const std::vector<tool>& tools) {
    vector<direction_info> norms =
      select_cut_directions(stock, part, f, tools);

    vector<direction_process_info> dir_info =
      initial_decompositions(stock, part, tools, norms);

    check_feature_depths(dir_info);

    return dir_info;
  }

  int curve_count(feature_decomposition* f) {
    int count = 0;

    for (auto feat : collect_features(f)) {
      count += curve_count(*feat);
    }

    return count;
  }

  int outer_curve_count(feature_decomposition* f) {
    DBG_ASSERT(f->num_children() == 1);

    int count = 0;

    cout << "Checking for outer features in " << normal(f) << endl;

#ifdef VIZ_DBG
    vtk_debug_feature_decomposition(f);
#endif
    
    feature_decomposition* top = f->child(0);
    vector<point> stock_ring = top->feature()->base().vertices();
    polygon_3 stock_polygon = build_clean_polygon_3(stock_ring);

    for (auto feat : collect_features(f)) {
      if (is_outer(*feat, stock_polygon) && feat->is_through()) {
	cout << "Found outer feature in " << normal(f) << endl;
	count += curve_count(*feat);
      }
    }

    cout << "Final curve count for " << normal(f) << " = " << count << endl;

    return count;
  }

  
  boost::optional<direction_process_info>
  find_outer_curve(std::vector<direction_process_info>& dir_info) {
    int curve_max = 0;
    unsigned max_index = 0;
    boost::optional<direction_process_info> outer_curve = boost::none;

    for (unsigned i = 0; i < dir_info.size(); i++) {
      auto dir = dir_info[i];
      auto decomp = dir.decomp;

      int outer_max = outer_curve_count(decomp);

      if (outer_max > curve_max) {
	outer_curve = dir;
	max_index = i;
	curve_max = outer_max;
      }
    }

    if (curve_max > 0) {
      direction_process_info next_elem = dir_info[max_index];

      cout << "Max curve count is " << curve_max << " in " << normal(next_elem.decomp) << endl;
      cout << "Max index is " << max_index << endl;

      dir_info.erase(begin(dir_info) + max_index);

      return next_elem;
    }

    return boost::none;
  }

}
