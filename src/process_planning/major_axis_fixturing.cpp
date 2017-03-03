#include "process_planning/major_axis_fixturing.h"

#include "backend/chamfer_operation.h"
#include "backend/drilled_hole_operation.h"
#include "backend/slice_roughing_operation.h"
#include "backend/freeform_toolpaths.h"
#include "feature_recognition/vertical_wall.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_selection.h"
#include "process_planning/feature_to_pocket.h"
#include "process_planning/job_planning.h"
#include "process_planning/tool_access.h"
#include "simulators/mill_tool.h"
#include "simulators/visual_debug.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/millability.h"
#include "synthesis/workpiece_clipping.h"
#include "synthesis/visual_debug.h"

namespace gca {

  boost::optional<dir_fixture>
  find_dir_fixture(const fixtures& f,
		   const point n,
		   const triangular_mesh& m) {
    Nef_polyhedron part_nef = trimesh_to_nef_polyhedron(m);

    vice v = f.get_vice();
    if (f.parallel_plates().size() > 0) {
      plate_height ph = f.parallel_plates().front();

      cout << "Selected parallel plate height = " << ph << endl;
      v = vice(f.get_vice(), ph);
    }

    vector<pair<clamp_orientation, homogeneous_transform>> orients =
      all_stable_orientations_with_side_transforms(part_nef, v, n);

    cout << "# of orients in " << n << " = " << orients.size() << endl;

    if (orients.size() > 0) {
      auto orient =
	max_e(orients,
	      [m]
	      (const std::pair<clamp_orientation, homogeneous_transform>& c)
	      { return c.first.contact_area(m); });

      cout << "Found fixture in " << n << endl;

      homogeneous_transform t = orient.second;

      return dir_fixture{orient.first, t, v};
    }

    return boost::none;
    
  }

  axis_fixture build_axis_fixture(const fixtures& f,
				  const major_axis_decomp& d) {
    const triangular_mesh& m = mesh(d);

    boost::optional<dir_fixture> positive =
      find_dir_fixture(f, d.major_axis, m);

    boost::optional<dir_fixture> negative =
      find_dir_fixture(f, -1*(d.major_axis), m);

    return axis_fixture{positive, negative};
  }

  triangular_mesh align_stock(const major_axis_decomp& cut_axis,
			      const dir_fixture& first_dir,
			      const workpiece w) {
    const auto& part = mesh(cut_axis);
    triangular_mesh m = stock_mesh(w);
    vector<surface> sfs = outer_surfaces(m);

    DBG_ASSERT(sfs.size() > 0);

    vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
    
    vector<plane> part_planes = cut_axis.axes;

    DBG_ASSERT(part_planes.size() == 3);

    vector<double> margin_values;
    for (auto& pl : part_planes) {
      if (angle_eps(pl.normal(), cut_axis.major_axis, 0.0, 1.0)) {
	margin_values.push_back(0.01);
      } else {
	margin_values.push_back(0.5);
      }
    }

    auto t = custom_offset_transform(part_planes,
				     stock_planes,
				     margin_values,
				     part,
				     m);

    DBG_ASSERT(t);

    triangular_mesh aligned_stock = apply(*t, m);
    
    //vtk_debug_meshes({part, aligned_stock});

    box part_bb = part.bounding_box();
    box aligned_bb = aligned_stock.bounding_box();

    DBG_ASSERT(fits_inside(part_bb, aligned_bb));

    return aligned_stock;
  }

  boost::optional<std::pair<fixture, homogeneous_transform> >
  find_next_fixture_side_vice(feature_decomposition* f,
			      const triangular_mesh& stock,
			      const point n,
			      const fixtures& fixes) {
    Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);
    return find_next_fixture_side_vice(f, stock_nef, stock, n, fixes);
  }


  void test_stock_volume(const Nef_polyhedron& stock_nef,
			 const triangular_mesh& part) {
    auto current_stock = nef_to_single_trimesh(stock_nef);
    cout << "Done with loop got current stock" << endl;

    double part_volume = volume(part);

    double stock_volume = volume(current_stock);
    double volume_ratio = part_volume / stock_volume;

    cout << "Part volume              = " << part_volume << endl;
    cout << "Final stock volume       = " << stock_volume << endl;
    cout << "Final part / stock       = " << volume_ratio << endl;

    // TODO: Tighten this tolerance once edge features are supported
    if (volume_ratio <= 0.99) {

      vtk_debug_mesh(part);
      vtk_debug_mesh(current_stock);

      DBG_ASSERT(false);
    }
  }

  void clean_features(std::vector<feature*>& feats,
		      const fixture& fix,
		      const tool_access_info& tool_info) {
    delete_if(feats, [tool_info](feature* f) {
	return map_find(f, tool_info).size() == 0;
      });

    auto unreachable_feats =
      unreachable_features(feats, fix);

    delete_if(feats, [&unreachable_feats](feature *f) {
	return elem(f, unreachable_feats);
      });

  }

  plane apply(const homogeneous_transform& t, const plane pl) {
    return plane(times_3(t.first, pl.normal()), apply(t, pl.pt()));
  }

  struct slice_setup {
    const triangular_mesh& wp_mesh;
    const triangular_mesh& part_mesh;
    const plane slice_plane;
    const std::vector<tool>& tools;
  };

  struct finishing_operations {
    std::vector<chamfer> chamfers;
    std::vector<freeform_surface> freeforms;
    feature_decomposition* decomp;
    tool_access_info access_info;

  public:

    std::vector<feature*> get_features() {
      return collect_features(decomp);
    }
  };

  fabrication_setup
  fixture_setup_to_fabrication_setup(fixture_setup& setup,
				     const material& stock_material) {
    auto toolpaths =
      cut_secured_mesh(setup.pockets, stock_material);
    return fabrication_setup(setup.arrangement(), setup.fix.v, toolpaths);
  }

  
  fixture_setup
  create_setup(const homogeneous_transform& s_t,
	       const slice_setup& slice_setup,
	       const fixture& f,
	       finishing_operations& finishing_ops) {

    auto chamfers = finishing_ops.chamfers;
    auto freeforms = finishing_ops.freeforms;

    auto aligned = apply(s_t, slice_setup.wp_mesh);
    auto part = apply(s_t, slice_setup.part_mesh);

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    triangular_mesh* pm = new (allocate<triangular_mesh>()) triangular_mesh(part);

    auto rotated_plane = apply(s_t, slice_setup.slice_plane);

    vector<pocket> pockets =
      feature_pockets(finishing_ops.get_features(), s_t, finishing_ops.access_info);
    //{slice_roughing_operation(rotated_plane, *m, *pm, slice_setup.tools)};

    

    for (auto& ch : chamfers) {
      pockets.push_back(chamfer_operation(ch.faces, part, ch.t));
    }

    for (auto& freeform : freeforms) {
      surface rotated_surf(pm, freeform.s.index_list());
      pockets.push_back(freeform_operation(rotated_surf, freeform.tools));
    }

    rigid_arrangement r;
    r.insert("part", *m);
    r.insert("final-part", *pm);


    return fixture_setup(r, f, pockets);
  }

  std::vector<toolpath>
  finish_toolpaths_for_feature(const feature& f,
			       const std::vector<tool>& tools,
			       const material& stock_material,
			       const double safe_z) {

    if (tools.size() == 0) {
      pair<double, double> feature_range = f.range_along(f.normal());
      cout << "ERROR: No available tools for feature" << endl;
      cout << "Feature depth = " << f.depth() << endl;
      cout << "Max along " << f.normal() << " = " << feature_range.second << endl;
      cout << "Min along normal " << f.normal() << " = " << feature_range.first << endl;
      vtk_debug_feature(f);
      DBG_ASSERT(tools.size() > 0);
    }

    labeled_polygon_3 base = f.base();
    point n = base.normal();

    if (!angle_eps(n, point(0, 0, 1), 0.0, 0.3)) {
      double theta = angle_between(n, point(0, 0, 1));
      cout << "base normal    = " << n << endl;
      cout << "desired normal = " << point(0, 0, 1) << endl;
      cout << "theta          = " << theta << endl;
      
      DBG_ASSERT(within_eps(theta, 0.0, 0.3));
    }

    DBG_ASSERT(f.depth() > 0.0);

    double base_z = base.vertex(0).z;
    double top_z = base_z + f.depth();

    if (!(base_z < top_z)) {
      cout << "base_z = " << base_z << endl;
      cout << "top_z  = " << top_z << endl;
      DBG_ASSERT(base_z < top_z);
    }

    point base_center = centroid(base.vertices());
    for (auto t : tools) {
      if (t.type() == TWIST_DRILL) {
	point up_direction(0, 0, 1);
	//DBG_ASSERT(false);
	//return {drilled_hole_operation(f.depth(), base_center, up_direction, t)};
	drilled_hole_operation drill_op(f.depth(), base_center, up_direction, t);
	return drill_op.make_toolpaths(stock_material, safe_z);
      }
    }

    oriented_polygon ob = to_oriented_polygon(base);

    vector<oriented_polygon> holes;
    for (auto h : base.holes()) {
      auto hp = oriented_polygon(n, h);

      holes.push_back(hp);
    }

    if (f.is_closed()) {
      //DBG_ASSERT(false);
      //return {flat_pocket(top_z, base_z, ob, holes, tools)};
      flat_pocket pocket_op(top_z, base_z, ob, holes, tools);
      return pocket_op.make_toolpaths(stock_material, safe_z);
    } else {
      DBG_ASSERT(!f.is_closed());

      if (holes.size() == 0) {
	face_pocket face(top_z, base_z, ob, tools);
	return {face.make_finish_toolpath(stock_material, safe_z)};
      }

      //DBG_ASSERT(false);
      //return {contour(top_z, base_z, base, tools)};
      contour contour_op(top_z, base_z, base, tools);
      return contour_op.make_finish_toolpaths(stock_material, safe_z);
    }
  }
  
  std::vector<toolpath>
  finish_prismatic_features(std::vector<feature*>& features,
			    const homogeneous_transform& t,
			    const tool_access_info& tool_info,
			    const material& stock_material,
			    const double safe_z) {
    vector<toolpath> toolpaths;
    for (auto f : features) {
      vector<tool> tools = map_find(f, tool_info);
      concat(toolpaths, finish_toolpaths_for_feature(f->apply(t), tools, stock_material, safe_z));
    }
    
    return toolpaths;
  }

  depth_field
  uniform_height_copy(const depth_field& other, const double z) {
    depth_field cpy(other.get_origin(),
		    other.x_len,
		    other.y_len,
		    other.resolution);

    for (int i = 0; i < cpy.num_x_elems; i++) {
      for (int j = 0; j < cpy.num_y_elems; j++) {
	cpy.set_column_height(i, j, z);
      }
    }

    return cpy;
  }

  double drop_tool(const point p,
		   const mill_tool& t,
		   const depth_field& r) {

    int first_x = r.x_index(t.x_min(p));
    int last_x = r.x_index(t.x_max(p));
    int first_y = r.y_index(t.y_min(p));
    int last_y = r.y_index(t.y_max(p));

    double highest = p.z;
    for (int i = first_x; i < last_x; i++) {
      for (int j = first_y; j < last_y; j++) {
	if (t.contains(p, r.get_origin(), r.resolution, i, j) &&
	    r.legal_column(i, j)) {
	  double test_pt = r.column_height(i, j);
	  if (test_pt >= highest) {
	    highest = test_pt;
	  }
	}
      }
    }

    return highest;
  }

  toolpath build_x_zig_path(const depth_field& part_field,
			    const tool& t) {

    DBG_ASSERT(t.type() == FLAT_NOSE);

    double safe_z = 10.0;

    vector<polyline> lines;
    cylindrical_bit mill_t(t.cut_diameter());

    for (unsigned i = 0; i < part_field.num_x_elems; i++) {
      double x_coord = part_field.x_coord(i);
      vector<point> pts;

      for (unsigned j = 0; j < part_field.num_y_elems; j++) {
	double y_coord = part_field.y_coord(j);
	point pt(x_coord, y_coord, 0.0);
	double z_coord = drop_tool(pt, mill_t, part_field); //part_field.column_height(i, j);

	pts.push_back(point(x_coord, y_coord, z_coord));
      }

      if (pts.size() > 1) {
	lines.push_back(pts);
      }
    }

    return {toolpath(FREEFORM_POCKET,
		     safe_z,
		     2000,
		     15.0,
		     7.5,
		     t,
		     lines)};
    
  }

  struct depth_layer {
    double z_min, z_max, z_upper_bound;
    std::vector<std::vector<std::pair<int, int> > > pts;
  };

  depth_field min_tool_height_field(const tool& t, const depth_field& part_field) {
    DBG_ASSERT(t.type() == FLAT_NOSE);

    depth_field min_height_field(part_field);

    double safe_z = 10.0;

    cylindrical_bit mill_t(t.cut_diameter());

    for (unsigned i = 0; i < part_field.num_x_elems; i++) {
      double x_coord = part_field.x_coord(i);
      vector<point> pts;

      for (unsigned j = 0; j < part_field.num_y_elems; j++) {
	double y_coord = part_field.y_coord(j);
	point pt(x_coord, y_coord, part_field.column_height(i, j));
	double z_coord = drop_tool(pt, mill_t, part_field);

	min_height_field.set_column_height(i, j, z_coord);
      }

    }

    return min_height_field;
    
  }

  depth_layer collect_layer(const double max_z,
			    const depth_field& df) {
    double z_min = max_z;
    double z_upper_bound = df.z_min() - 1.0;
    vector<vector<pair<int, int> > > pts;
    for (int i = 0; i < df.num_x_elems; i++) {
      double x = df.x_coord(i);

      vector<pair<int, int> > y_pts;
      for (int j = 0; j < df.num_y_elems; j++) {
	double y = df.y_coord(j);
	double z = df.column_height(i, j);

	if (z < max_z) {
	  y_pts.push_back(make_pair(i, j));

	  if (z > z_upper_bound) {
	    z_upper_bound = z;
	  }

	}

	if (z < z_min) {
	  z_min = z;
	}
      }

      if (y_pts.size() > 1) {
	pts.push_back(y_pts);
      }
    }

    return depth_layer{z_min, max_z, z_upper_bound, pts};
  }

  std::vector<depth_layer>
  slice_depth_layers(const tool& t,
		     const double max_height,
		     const depth_field& part_field) {
    DBG_ASSERT(max_height > part_field.z_max());

    depth_field d = min_tool_height_field(t, part_field);
    double max_h = max_height;

    vector<depth_layer> layers;
    while (max_h > d.z_min()) {
      depth_layer next = collect_layer(max_h, d);
      layers.push_back(next);
      max_h = next.z_upper_bound;
    }

    return layers;
  }

  std::vector<std::vector<std::pair<int, int> > >
  group_connected_lines(const std::vector<std::pair<int, int> >& pixels) {
    
  }

  toolpath
  toolpath_for_depth_layer(const depth_field& df,
			   const depth_layer& dl,
			   const tool& t) {

    vector<polyline> lines;
    for (auto& l : dl.pts) {
      auto connected_lines =
	group_connected_lines(l);

      for (auto& pix_line : connected_lines) {
	if (pix_line.size() > 0) {
	  vector<point> drop_points;
	  for (auto& pt : pix_line) {
	    double x = df.x_coord(pt.first);
	    double y = df.y_coord(pt.second);
	    drop_points.push_back(point(x, y, dl.z_upper_bound));
	  }

	  lines.push_back(drop_points);
	}
      }
    }

    return {toolpath(FREEFORM_POCKET,
		     dl.z_max,
		     2000,
		     15.0,
		     7.5,
		     t,
		     lines)};
    
  }

  std::vector<toolpath>
  build_roughing_paths(const depth_field& part_field,
		       const double max_height,
		       const tool& current_tool) {
    vector<depth_layer> depth_layers =
      slice_depth_layers(current_tool, max_height, part_field);

    cout << "# of layers = " << depth_layers.size() << endl;

    vector<toolpath> toolpaths;
    for (auto& layer : depth_layers) {
      toolpaths.push_back(toolpath_for_depth_layer(part_field, layer, current_tool));
    }

    return toolpaths;
  }

  std::vector<toolpath>
  roughing_toolpaths(const triangular_mesh& stock,
		     const triangular_mesh& part,
		     const std::vector<tool>& tools) {
    double field_resolution = 0.05;
    depth_field part_field = build_from_stl(part, field_resolution);

    double stock_height = max_in_dir(stock, point(0, 0, 1));
    depth_field current_heights = uniform_height_copy(part_field, stock_height);

    vector<tool> flat_tools =
      select(tools, [](const tool& t) { return t.type() == FLAT_NOSE; });

    cout << "# of flat tools = " << flat_tools.size() << endl;
    
    DBG_ASSERT(flat_tools.size() > 0);

    sort_gt(flat_tools, [](const tool& t) { return t.cut_diameter(); });

    // vtk_debug_depth_field(current_heights);
    // vtk_debug_depth_field(part_field);

    double z_max = current_heights.z_max();
    vector<toolpath> rough_paths;
    for (auto& current_tool : flat_tools) {
      vector<toolpath> max_toolpaths =
	build_roughing_paths(part_field, z_max, current_tool);
      concat(rough_paths, max_toolpaths);
    }

    return rough_paths;
  }

  fabrication_setup
  create_fab_setup(const homogeneous_transform& s_t,
		   const slice_setup& slice_setup,
		   const fixture& f,
		   finishing_operations& finishing_ops,
		   const material& stock_material) {

    auto chamfers = finishing_ops.chamfers;
    auto freeforms = finishing_ops.freeforms;

    auto aligned = apply(s_t, slice_setup.wp_mesh);
    auto part = apply(s_t, slice_setup.part_mesh);

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    triangular_mesh* pm = new (allocate<triangular_mesh>()) triangular_mesh(part);

    double safe_z = max_in_dir(aligned, point(0, 0, 1)) + 0.2;

    auto rotated_plane = apply(s_t, slice_setup.slice_plane);

    vector<feature*> feats = finishing_ops.get_features();
    vector<toolpath> toolpaths = roughing_toolpaths(aligned, part, slice_setup.tools);
    vector<toolpath> finish_toolpaths =
      finish_prismatic_features(feats,
				s_t,
				finishing_ops.access_info,
				stock_material,
				safe_z);
    concat(toolpaths, finish_toolpaths);
    //{slice_roughing_operation(rotated_plane, *m, *pm, slice_setup.tools)};

    for (auto& ch : chamfers) {
      chamfer_operation ch_op(ch.faces, part, ch.t);
      std::vector<toolpath> finish = ch_op.make_toolpaths(stock_material, safe_z);
      concat(toolpaths, finish);
    }

    for (auto& freeform : freeforms) {
      surface rotated_surf(pm, freeform.s.index_list());
      freeform_operation freeform_op(rotated_surf, freeform.tools);
      toolpath finish = freeform_op.make_finish_toolpath(stock_material, safe_z);
      toolpaths.push_back(finish);
    }

    rigid_arrangement r;
    r.insert("part", *m);
    r.insert("final-part", *pm);

    return fabrication_setup(r, f.v, toolpaths);
  }
  

  void delete_inaccessable_features(feature_decomposition* decomp,
				    const tool_access_info& ti) {
    auto no_tools = [ti](feature* f) {
      return map_find(f, ti).size() == 0;
    };

    delete_nodes(decomp, no_tools);
  }

  void delete_features_below_plane(feature_decomposition* decomp,
				    const plane slice_plane) {
    point n = slice_plane.normal();
    point decomp_n = normal(decomp);

    DBG_ASSERT(angle_eps(n, decomp_n, 0.0, 1.0));

    auto below_plane = [slice_plane](feature* f) {
      point n = slice_plane.normal();
      point base = slice_plane.pt();

      return signed_distance_along(f->base().vertex(0), n) + 0.001 <
      signed_distance_along(base, n);
    };

    delete_nodes(decomp, below_plane);
  }
  
  void check_feature_depths(feature_decomposition* decomp) {
    for (auto& f : collect_features(decomp)) {
      DBG_ASSERT(f->depth() > 0.0);
    }
  }

  finishing_operations
  build_finishing_ops(const triangular_mesh& stock,
		      const triangular_mesh& part,
		      const plane slice_plane,
		      const std::vector<tool>& tools) {
    point n = slice_plane.normal();

    // TODO: Add access testing
    vector<chamfer> chamfers = chamfer_regions(part, n, tools);
    vector<freeform_surface> freeform_surfs;
    freeform_surfs = freeform_surface_regions(part, n, tools);

    feature_decomposition* decomp =
      build_feature_decomposition(stock, part, n);
    delete_features_below_plane(decomp, slice_plane);
    
    tool_access_info ti = find_accessable_tools(decomp, tools);
    delete_inaccessable_features(decomp, ti);

    check_feature_depths(decomp);

    return finishing_operations{chamfers, freeform_surfs, decomp, ti};
  }

  slice_setup build_roughing_ops(const triangular_mesh& stock,
				 const triangular_mesh& part,
				 const plane slice_plane,
				 const std::vector<tool>& tools) {
    return slice_setup{stock, part, slice_plane, tools};
  }

  fabrication_setup
  build_second_setup(const plane slice_plane,
		     const triangular_mesh& part,
		     const triangular_mesh& stock,
		     const dir_fixture& second_dir,
		     const std::vector<tool>& tools,
		     const material& mat) {
    fixture fix(second_dir.orient, second_dir.v);

    point n = second_dir.orient.top_normal();

    slice_setup rough_ops = build_roughing_ops(stock, part, slice_plane, tools);
    finishing_operations finish_ops =
      build_finishing_ops(stock, part, slice_plane, tools);
    
    fabrication_setup s =
      create_fab_setup(second_dir.placement,
		       rough_ops,
		       fix,
		       finish_ops,
		       mat);

    return s;
  }

  fabrication_plan
  axis_fabrication_plan(const major_axis_decomp& cut_axis,
  			const axis_fixture& axis_fix,
  			const fixtures& fixes,
  			const workpiece w,
  			const std::vector<tool>& tools) {

    dir_fixture first_dir = *(axis_fix.positive);
    dir_fixture second_dir = *(axis_fix.negative);

    triangular_mesh stock = align_stock(cut_axis, first_dir, w);
    vector<fabrication_setup> setups;

    const auto& part = mesh(cut_axis);

    // NOTE: Assumes the base of the part is above the vice
    point n = first_dir.orient.top_normal();
    point pt = min_point_in_dir(part, n);
    plane slice_plane(n, pt);

    Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);
    double depth = signed_distance_along(slice_plane.pt(), slice_plane.normal());
    auto maybe_fix = find_next_fixture_side_vice(depth, stock_nef, stock, n, fixes);
    
    DBG_ASSERT(maybe_fix);

    slice_setup rough_ops = build_roughing_ops(stock, part, slice_plane, tools);
    finishing_operations finish_ops =
      build_finishing_ops(stock, part, slice_plane, tools);
    
    fabrication_setup s =
      create_fab_setup(maybe_fix->second,
		       rough_ops,
		       maybe_fix->first,
		       finish_ops,
		       w.stock_material);

    setups.push_back(s);

    fabrication_setup second =
      build_second_setup(slice_plane.flip(), part, stock, second_dir, tools, w.stock_material);

    setups.push_back(second);

    return fabrication_plan(setups);
  }
  
}
