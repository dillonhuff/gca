#include "backend/chamfer_operation.h"
#include "geometry/extrusion.h"
#include "geometry/mesh_operations.h"
#include "geometry/offset.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/axis_location.h"
#include "process_planning/feature_selection.h"
#include "process_planning/feature_to_pocket.h"
#include "process_planning/job_planning.h"
#include "synthesis/visual_debug.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/check.h"

//#define VIZ_DBG

namespace gca {

  point part_zero_position(const fixture& f) {
    DBG_ASSERT(f.has_part_zero());

    clamp_orientation clamp_orient = f.get_zero_planes();

    return part_zero_position(clamp_orient);
  }

  fixture shift_vice(const point p, const fixture& f) {
    fixture shifted = f;
    shifted.v.set_position(f.v.position() + p);

    return shifted;
  }

  fixture_setup
  create_setup(const homogeneous_transform& s_t,
	       const triangular_mesh& wp_mesh,
	       const triangular_mesh& part_mesh,
	       const std::vector<feature*>& features,
	       const fixture& f,
	       const tool_access_info& tool_info,
	       const std::vector<chamfer>& chamfers) {

    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);

    vector<pocket> pockets = feature_pockets(features, s_t, tool_info);
    for (auto& ch : chamfers) {
      pockets.push_back(chamfer_operation(ch.faces, part, ch.t));
    }

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  triangular_mesh feature_mesh(const feature& f,
			       const double base_dilation,
			       const double top_extension,
			       const double base_extension) {
    point shift_vec = (-1*base_extension)*f.normal();

    if (base_dilation > 0.0) {
      auto dilated_base = dilate(f.base(), base_dilation);
      dilated_base = shift(shift_vec, dilated_base);
    
      auto m = extrude(dilated_base, (top_extension + f.depth())*f.normal());

      DBG_ASSERT(m.is_connected());

      return m;

    } else {
      auto base = shift(shift_vec, f.base());

      auto m = extrude(base, (top_extension + f.depth())*f.normal());

      DBG_ASSERT(m.is_connected());

      return m;
    }
  }

  triangular_mesh feature_mesh(const feature& f) {

    return feature_mesh(f, 0.0, 0.0001, 0.0000); 
  }

  Nef_polyhedron
  subtract_surface(const Nef_polyhedron& stock_nef,
		   const std::vector<index_t>& surf,
		   const triangular_mesh& part,
		   const point n) {
    triangular_mesh m = extrude_surface_negative(surf, part, n, 200);

    //vtk_debug_meshes({m, part});

    Nef_polyhedron mesh_nef = trimesh_to_nef_polyhedron(m);

    DBG_ASSERT(mesh_nef.is_simple());

    return stock_nef - mesh_nef;
  }
  
  Nef_polyhedron
  subtract_chamfers(const Nef_polyhedron& stock_nef,
		    const std::vector<chamfer>& chamfers,
		    const triangular_mesh& part,
		    const point n) {
    if (chamfers.size() == 0) {
      return stock_nef;
    }

    Nef_polyhedron final_nef = stock_nef;
    for (auto& s : chamfers) {

      final_nef = subtract_surface(final_nef, s.faces, part, n);
    }

    return final_nef;
  }

  Nef_polyhedron
  subtract_features(const Nef_polyhedron& m,
		    const std::vector<feature*>& features) {

    auto res = m;
    for (auto f : features) {
      Nef_polyhedron f_nef =
	trimesh_to_nef_polyhedron(feature_mesh(*f, 0.0000001 /*1*/, 1.0, 0.000001));

      //1)); //0.0001));

      for (auto it = f_nef.volumes_begin(); it != f_nef.volumes_end(); it++) {
	cout << "VOLUME" << endl;
      }

      res = (res - f_nef).regularization();
      
    }

    return res;
    
  }

  double volume(const feature& f) {
    const rotation r = rotate_from_to(f.normal(), point(0, 0, 1));
    auto bp = to_boost_poly_2(apply(r, f.base()));

    double base_area = bg::area(bp);
    
    return base_area * f.depth();
  }

  struct volume_info {
    double volume;

    Nef_polyhedron remaining_volume;

    Nef_polyhedron dilated_mesh;
  };

  typedef std::unordered_map<feature*, volume_info> volume_info_map;

  volume_info initial_volume_info(const feature& f,
				  const Nef_polyhedron& stock_nef) {
    triangular_mesh mesh = feature_mesh(f);
    auto feature_nef = trimesh_to_nef_polyhedron(mesh);
    feature_nef = stock_nef.intersection(feature_nef);

    cout << "Got undilated feature mesh" << endl;

    double vol = volume(nef_to_single_trimesh(feature_nef));

    //	TODO: Refine the dilation tolerance, it may not matter but
    //	best to be safe
    triangular_mesh dilated_mesh = feature_mesh(f, 0.00005, 0.05, 0.0); //0.000005, 0.05, 0.0);

    return volume_info{vol, feature_nef, trimesh_to_nef_polyhedron(dilated_mesh)};
  }

  volume_info
  update_volume_info(const volume_info& inf,
		     const std::vector<Nef_polyhedron>& to_subtract) {
    if (inf.volume == 0.0) { return inf; }

    Nef_polyhedron res = inf.remaining_volume;
    for (auto s : to_subtract) {
      res = res - s;
    }

    double new_volume = 0.0;
    for (auto& m : nef_polyhedron_to_trimeshes(res)) {
      new_volume += volume(m);
    }

    cout << "Old volume = " << inf.volume << endl;
    cout << "New volume = " << new_volume << endl;

    return volume_info{new_volume, res, inf.dilated_mesh};
  }

  volume_info_map
  initial_volume_info(const std::vector<direction_process_info>& dir_info,
		      const Nef_polyhedron& stock_nef) {
    volume_info_map m;

    for (auto d : dir_info) {
      auto decomp = d.decomp;
      for (feature* f : collect_features(decomp)) {
	m[f] = initial_volume_info(*f, stock_nef);
      }
    }

    return m;
  }

  // NOTE: Perhaps all we need to do is find a viable hull for the remaining feature?
  boost::optional<feature>
  extract_feature(const feature& original,
		  const triangular_mesh& portion) {

    point n = original.normal();

    cout << "NORMAL = " << n << endl;

    //vtk_debug_mesh(portion);

    vector<index_t> inds = portion.face_indexes();

    auto top = select(inds, [portion, n](const index_t i) {
    	return angle_eps(portion.face_orientation(i), n, 0.0, 1.0);
      });

    auto top_cpy = top;
    auto top_regions = normal_delta_regions(top_cpy, portion, 180.0);

    // cout << "TOP" << endl;
    // cout << portion.face_orientation(top.front()) << endl;
    // vtk_debug_highlight_inds(top, portion);

    if (top_regions.size() != 1) {
      cout << "# of top regions = " << top_regions.size() << endl;
      return boost::none;
    }

    subtract(inds, top);

    auto bottom = select(inds, [portion, n](const index_t i) {
	return angle_eps(portion.face_orientation(i), n, 180.0, 1.0);
      });

    auto bottom_cpy = bottom;
    auto bottom_regions = normal_delta_regions(bottom_cpy, portion, 180.0);

    // cout << "BOTTOM" << endl;
    // cout << portion.face_orientation(bottom.front()) << endl;
    // vtk_debug_highlight_inds(bottom, portion);

    if (bottom_regions.size() != 1) {
      cout << "# of bottom regions = " << bottom_regions.size() << endl;
      return boost::none;
    }

    subtract(inds, bottom);

    // cout << "REST" << endl;
    // vtk_debug_highlight_inds(inds, portion);

    bool all_other_triangles_vertical =
      all_of(begin(inds), end(inds), [portion, n](const index_t i) {
	  return angle_eps(portion.face_orientation(i), n, 90.0, 1.0);
	});

    if (!all_other_triangles_vertical) {
      cout << "All other triangles not vertical" << endl;
      return boost::none;
    }

    vector<oriented_polygon> bounds = mesh_bounds(bottom, portion);
    vector<vector<point>> rings;
    for (auto& b : bounds) {
      rings.push_back(b.vertices());
    }

    vector<polygon_3> polys = arrange_rings(rings);


    if (polys.size() != 1) {
      // vtk_debug_polygons(polys);
      // DBG_ASSERT(polys.size() == 1);

      DBG_ASSERT(polys.size() > 0);
      return boost::none;
    }

    cout << "Using split feature" << endl;
    if (polys.front().holes().size() == original.base().holes().size()) {
      feature clipped_feature(original.is_closed(), original.depth(), polys.front());
      return clipped_feature;
    } else {
      feature clipped_feature(true, original.depth(), polys.front());
      return clipped_feature;
    }

  }

  std::vector<feature*>
  clipped_features(feature* f,
		   const volume_info& vol_info,
		   tool_access_info& tool_info) {
    auto& feat_nef = vol_info.remaining_volume;
    vector<triangular_mesh> meshes = nef_polyhedron_to_trimeshes(feat_nef);

    vector<feature*> clipped_features;
    for (auto& feature_mesh : meshes) {
      boost::optional<feature> f_m = extract_feature(*f, feature_mesh);

      if (f_m) {
	clipped_features.push_back(new (allocate<feature>()) feature(*f_m));
      } else {
	return {f};
      }

    }

    for (auto cf : clipped_features) {
      tool_info[cf] = map_find(f, tool_info);
    }

    return clipped_features;
  }

  void
  clip_volumes(std::vector<feature*>& feats_to_sub,
	       volume_info_map& volume_inf,
	       const std::vector<direction_process_info>& dir_info) {
    vector<feature*> feats_left;
    for (auto d : dir_info) {
      concat(feats_left, collect_features(d.decomp));
    }

    vector<Nef_polyhedron> to_subtract;
    for (auto f : feats_to_sub) {
      volume_info f_info = map_find(f, volume_inf);

      // If the volume still exists
      if (f_info.volume > 0.0) {
	to_subtract.push_back(f_info.dilated_mesh);
      }
    }

    for (auto& info_pair : volume_inf) {
      feature* f = info_pair.first;

      // Do not subtract the selected decomposition, those
      // features are being removed anyway
      if (elem(f, feats_left) && !elem(f, feats_to_sub)) {
	volume_inf[f] = update_volume_info(info_pair.second, to_subtract);
      }
    }
  }

  double volume(feature* f, const volume_info_map& vol_info) {
    volume_info inf = map_find(f, vol_info);

    return inf.volume;
  }

  double volume(feature_decomposition* f, const volume_info_map& vol_info) {
    auto feats = collect_features(f);

    double vol = 0.0;
    for (auto ft : feats) {
      vol += volume(ft, vol_info);
    }

    return vol;
  }

  int curve_count(const std::vector<point>& ring) {
    int count = 0;

    unsigned num_pts = ring.size();
    for (unsigned i = 0; i < num_pts; i++) {
      unsigned prev = (i + (num_pts - 1)) % num_pts;
      unsigned next = (i + 1) % num_pts;

      point prev_pt = ring[prev];
      point current_pt = ring[i];
      point next_pt = ring[next];

      point d1 = current_pt - prev_pt;
      point d2 = next_pt - current_pt;

      if (!(angle_eps(d1, d2, 0.0, 3.0) ||
	    angle_eps(d1, d2, 90.0, 3.0) ||
	    angle_eps(d1, d2, -90.0, 3.0) ||
	    angle_eps(d1, d2, 180.0, 3.0))) {
	count++;
      }
    }

    return count;
  }

  int curve_count(const polygon_3& f) {
    int count = 0;
    count += curve_count(f.vertices());

    for (auto& h : f.holes()) {
      count += curve_count(h);
    }

    return count;
  }

  int curve_count(const feature& f) {
    return curve_count(f.base());
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
      if (is_outer(*feat, stock_polygon)) {
	cout << "Found outer feature in " << normal(f) << endl;
	count += curve_count(*feat);
      }
    }

    cout << "Final curve count for " << normal(f) << " = " << count << endl;

    return count;
  }

  int curve_count(feature_decomposition* f) {
    int count = 0;

    for (auto feat : collect_features(f)) {
      count += curve_count(*feat);
    }

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

  direction_process_info
  select_next_dir(std::vector<direction_process_info>& dir_info,
		  const volume_info_map& vol_info) {
    cout << "Selecting next direction" << endl;
    cout << "Candidates" << endl;
    for (auto d : dir_info) {
      cout << normal(d.decomp) << " with volume " << volume(d.decomp, vol_info) << endl;
    }

    DBG_ASSERT(dir_info.size() > 0);

    auto outer_curve = find_outer_curve(dir_info);

    if (outer_curve) {
      cout << "Found outer curve" << endl;
      cout << "Selected direction " << normal(outer_curve->decomp) << endl;
      return *outer_curve;
    }

    cout << "No relevant outer curve" << endl;

    
    auto next = max_element(begin(dir_info), end(dir_info),
    			    [vol_info](const direction_process_info& l,
    				       const direction_process_info& r) {
    			      return volume(l.decomp, vol_info) <
    			      volume(r.decomp, vol_info);
    			    });

    DBG_ASSERT(next != end(dir_info));

    direction_process_info next_elem = *next;

    dir_info.erase(next);

    cout << "Selected direction " << normal(next_elem.decomp) << endl;
    cout << "Volume of direction = " << volume(next_elem.decomp, vol_info) << endl;

    return next_elem;
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

  std::vector<direction_process_info>
  initial_decompositions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const std::vector<tool>& tools,
			 const std::vector<point>& norms) {

    //    vector<double> angles = chamfer_angles(tools);

    vector<direction_process_info> dir_info;
    for (auto n : norms) {
      feature_decomposition* decomp = build_feature_decomposition(stock, part, n);
      tool_access_info info = find_accessable_tools(decomp, tools);
      vector<chamfer> chamfers = chamfer_regions(part, n, tools);
      dir_info.push_back({decomp, info, chamfers});
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

  std::vector<feature*>
  collect_viable_features(feature_decomposition* decomp,
			  const volume_info_map& vol_info,
			  const fixture& fix) {
    std::vector<feature*> fs = collect_features(decomp);

    cout << "# of features initially = " << fs.size() << endl;

    delete_if(fs, [vol_info](feature* feat) {
	return map_find(feat, vol_info).volume < 0.00001;
      });

    cout << "# of features with some volume left = " << fs.size() << endl;

    auto unreachable_feats =
      unreachable_features(fs, fix);

    delete_if(fs, [&unreachable_feats](feature *f) {
	return elem(f, unreachable_feats);
      });

    cout << "# of features after deleting the unreachable = " << fs.size() << endl;

    return fs;

  }

  boost::optional<double>
  select_parallel_plate(feature_decomposition* decomp,
			const triangular_mesh& current_stock,
			const fixtures& f) {

    DBG_ASSERT(decomp->num_children() == 1);

    feature_decomposition* top = decomp->child(0);
    vector<point> stock_ring = top->feature()->base().vertices();
    polygon_3 stock_polygon = build_clean_polygon_3(stock_ring);

    point n = normal(decomp);

    // TODO: Should really make this max double value
    double min_outer_depth = 1e25;

    for (auto feat : collect_features(decomp)) {
      if (is_outer(*feat, stock_polygon)) {
	double feat_min = feat->min_distance_along(n);

	if (feat_min < min_outer_depth) {
	  min_outer_depth = feat_min;
	}

      }
    }

    vice v_pre = f.get_vice();
    double part_min = min_in_dir(current_stock, n);

    // TODO: Make this a parameter rather than a magic number?
    double safe_margin = 0.05;

    if (min_outer_depth > (part_min + v_pre.jaw_height() + safe_margin)) {
      return boost::none;
    }

    vector<double> heights = f.parallel_plates();
    sort(begin(heights), end(heights));

    for (auto height : heights) {
      vice test_v(v_pre, height);

      if (min_outer_depth > (part_min + test_v.jaw_height() + safe_margin)) {
	return height;
      }
      
    }
    
    double max_plate = max_e(f.parallel_plates(), [](const plate_height p)
			     { return p; });

    return max_plate;
  }

  clamp_orientation
  find_part_zero(const Nef_polyhedron& part_nef,
		 const clamp_orientation& orient,
		 const vice& v,
		 const point n) {
    auto part = nef_to_single_trimesh(part_nef);

    // polygon_3 hull = convex_hull_2D(part.vertex_list(), n, 0.0);

    // point vice_pl_pt = min_point_in_dir(part, n) + v.jaw_height()*n;
    // plane vice_top_plane(-1*n, vice_pl_pt);

    // double big = 2000.0;

    // polygon_3 p = project_onto(vice_top_plane, hull);
    // polygon_3 plane_approx = dilate(p, big);

    // triangular_mesh m = extrude(plane_approx, big*n);
    
    // auto clip_nef = trimesh_to_nef_polyhedron(m);
    // auto cut_parts_nef = part_nef - clip_nef;

    // auto cut_part = nef_to_single_trimesh(cut_parts_nef); //cut_parts.front();

    // vector<surface> cregions = outer_surfaces(cut_part);
    // sort(begin(cregions), end(cregions),
    // 	 [](const surface& l, const surface& r)
    // 	 { return l.surface_area() < r.surface_area(); });
    // reverse(begin(cregions), end(cregions));


    // std::vector<clamp_orientation> orients =
    //   all_stable_orientations_with_top_normal(cregions, v, n);

    // DBG_ASSERT(orients.size() > 0);

    // auto orient = max_e(orients, [part](const clamp_orientation& c)
    // 			{ return c.contact_area(part); });

    // vector<surface> initial_regions =
    //   inds_to_surfaces(const_orientation_regions(part), part);

    // vector<unsigned> millable_from =
    //   surfaces_millable_from(orient, ptrs(initial_regions), v);

    // vector<surface> cregions;
    // for (auto ind : millable_from) {
    //   cregions.push_back(initial_regions[ind]);
    // }

    //    vtk_debug_highlight_inds(initial_regions);

    // vector<surface> cregions =
    //   surfaces_visible_from(initial_regions, n);

    vector<surface> cregions = outer_surfaces(part);

    vtk_debug_highlight_inds(cregions);

    DBG_ASSERT(cregions.size() > 2);

    std::vector<surface> ortho =
      take_basis(cregions, [](const surface& l, const surface& r) {
    	  return angle_eps(normal(l), normal(r), 90.0, 1.0);
    	},
    	3);

    clamp_orientation zero_planes(&(ortho[0]), &(ortho[1]), &(ortho[2]));
    return zero_planes;
  }

  boost::optional<std::pair<fixture, homogeneous_transform>>
  find_next_fixture(feature_decomposition* decomp,
		    Nef_polyhedron& stock_nef,
		    const triangular_mesh& current_stock,
		    const point n,
		    const fixtures& f) {

    boost::optional<double> par_plate =
      select_parallel_plate(decomp, current_stock, f);

    vice v = f.get_vice();
    if (par_plate) {
      v = vice(f.get_vice(), *par_plate);
    }

    auto orients = all_stable_orientations_box(stock_nef, v, n);
    cout << "# of orients in " << n << " = " << orients.size() << endl;

    if (orients.size() > 0) {
      auto orient = max_e(orients, [current_stock](const clamp_orientation& c)
			  { return c.contact_area(current_stock); });

      cout << "Found fixture in " << n << endl;

      fixture fix(orient, v);

      homogeneous_transform t =
	balanced_mating_transform(current_stock, fix.orient, fix.v);

      return std::make_pair(fix, t);
    }

    return boost::none;
  }

  boost::optional<std::pair<fixture, homogeneous_transform> >
  find_next_fixture_side_vice(feature_decomposition* decomp,
			      Nef_polyhedron& stock_nef,
			      const triangular_mesh& current_stock,
			      const point n,
			      const fixtures& f) {

    boost::optional<double> par_plate =
      select_parallel_plate(decomp, current_stock, f);

    vice v = f.get_vice();
    if (par_plate) {
      v = vice(f.get_vice(), *par_plate);
    }

    vector<pair<clamp_orientation, homogeneous_transform>> orients =
      all_stable_orientations_with_side_transforms(stock_nef, v, n);

    cout << "# of orients in " << n << " = " << orients.size() << endl;

    if (orients.size() > 0) {
      auto orient =
	max_e(orients,
	      [current_stock]
	      (const std::pair<clamp_orientation, homogeneous_transform>& c)
	      { return c.first.contact_area(current_stock); });

      cout << "Found fixture in " << n << endl;

      fixture fix(orient.first, v);

      homogeneous_transform t = orient.second;

      return std::make_pair(fix, t);
    }

    return boost::none;
  }  
  std::vector<fixture_setup>
  select_jobs_and_features(const triangular_mesh& stock,
			   const triangular_mesh& part,
			   const fixtures& f,
			   //			   const std::vector<tool>& tools,
			   std::vector<direction_process_info>& dir_info) {
			   //			   const std::vector<point>& norms) {

    Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);

    volume_info_map volume_inf = initial_volume_info(dir_info, stock_nef);

    vector<fixture_setup> cut_setups;

    double part_volume = volume(part);

#ifdef VIZ_DBG
    // for (auto d : dir_info) {
    //   vtk_debug_feature_decomposition(d.decomp);
    // }

    vector<feature*> init_features;
    cout << "# of directions = " << dir_info.size() << endl;
    for (auto d : dir_info) {
      concat(init_features, collect_features(d.decomp));
    }

    vtk_debug_features(init_features);
#endif

    vector<feature*> all_features{};

    while (dir_info.size() > 0) {
      direction_process_info info = select_next_dir(dir_info, volume_inf);

      cout << "Trying direction " << normal(info.decomp) << endl;

      cout << "In loop getting current stock" << endl;
      auto current_stock = nef_to_single_trimesh(stock_nef);
      cout << "In loop got current stock" << endl;

      point n = normal(info.decomp);

      boost::optional<std::pair<fixture, homogeneous_transform>> maybe_fix =
	find_next_fixture_side_vice(info.decomp, stock_nef, current_stock, n, f);

#ifdef VIZ_DBG
      vtk_debug_feature_decomposition(info.decomp);
#endif

      if (maybe_fix) {
	fixture fix = maybe_fix->first;

	auto decomp = info.decomp;
	auto& acc_info = info.tool_info;

	homogeneous_transform t = maybe_fix->second;
	auto features = collect_viable_features(decomp, volume_inf, fix);

	cout << "# of viable features = " << features.size() << endl;

#ifdef VIZ_DBG
	fabrication_setup dummy(apply(t, current_stock), fix.v, {});
	visual_debug(dummy);
#endif

	clip_volumes(features, volume_inf, dir_info);

	if (features.size() > 0) {

	  cout << "Found features in " << n << endl;

#ifdef VIZ_DBG
	  for (auto f : features) {
	    auto vol_data = map_find(f, volume_inf);
	    cout << "delta volume = " << vol_data.volume << endl;

	    vtk_debug_feature(*f);

	    vtk_debug_meshes(nef_polyhedron_to_trimeshes(vol_data.remaining_volume));

	  }
	  vtk_debug_features(features);
	  concat(all_features, features);
#endif

	  vector<feature*> final_features;
	  for (auto f : features) {
	    concat(final_features,
		   clipped_features(f, map_find(f, volume_inf), info.tool_info));
	  }

	  cut_setups.push_back(create_setup(t, current_stock, part, final_features, fix, info.tool_info, info.chamfer_surfaces));

	  stock_nef = subtract_features(stock_nef, features);
	  // NOTE: Assumes all chamfers are accessable
	  stock_nef = subtract_chamfers(stock_nef, info.chamfer_surfaces, part, n);

	  cout << "Just before in loop nef to trimesh" << endl;
	  current_stock = nef_to_single_trimesh(stock_nef);
	  cout << "Just after in loop nef to trimesh" << endl;

	  double stock_volume = volume(current_stock);
	  double volume_ratio = part_volume / stock_volume;

	  cout << "Part volume  = " << part_volume << endl;
	  cout << "Stock volume = " << stock_volume << endl;
	  cout << "part / stock = " << volume_ratio << endl;

#ifdef VIZ_DBG
	  vtk_debug_mesh(current_stock);
#endif
	  if (volume_ratio > 0.999) { break; }
	}
      }
    }

    cout << "Done with loop getting current stock" << endl;
    auto current_stock = nef_to_single_trimesh(stock_nef);
    cout << "Done with loop got current stock" << endl;

    double stock_volume = volume(current_stock);
    double volume_ratio = part_volume / stock_volume;

    cout << "Part volume              = " << part_volume << endl;
    cout << "Final stock volume       = " << stock_volume << endl;
    cout << "Final part / stock       = " << volume_ratio << endl;

#ifdef VIZ_DBG
    vtk_debug_features(all_features);
#endif

    // TODO: Tighten this tolerance once edge features are supported
    if (volume_ratio <= 0.99) {

      vtk_debug_mesh(part);
      vtk_debug_mesh(current_stock);

      DBG_ASSERT(false);
    }

    return cut_setups;
  }

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools) {
    vector<point> norms = select_cut_directions(stock, part, f, tools);
    vector<direction_process_info> dir_info =
      initial_decompositions(stock, part, tools, norms);

    //return select_jobs_and_features(stock, part, f, tools, dir_info);
    return select_jobs_and_features(stock, part, f, dir_info);
  }

}
