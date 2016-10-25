#include <cmath>

#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "gcode/cut.h"
#include "gcode/linear_cut.h"
#include "geometry/mesh_operations.h"
#include "geometry/offset.h"
#include "geometry/vtk_debug.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/shape_layout.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "utils/algorithm.h"

namespace gca {

  struct cut_move_parameters {
    double feed;
    double plunge_feed;
    double speed;
    double cut_depth;
  };

  cut_move_parameters
  calculate_cut_params_aluminum(const tool& t,
				const pocket_name op_name) {
    double cut_depth, speed, feed;
    
    cut_depth = t.cut_diameter() / 3.0;

    if (t.cut_diameter() < 0.25) {
      speed = 2500;
    } else {
      speed = 1000;
    }

    feed = 5.0;

    cout << "Chip Load Per Tooth = " << chip_load_per_tooth(t, feed, speed) << endl;

    return cut_move_parameters{feed, feed / t.num_flutes(), speed, cut_depth};
  }
  
  // TODO: More detailed cut parameter calculation
  cut_move_parameters calculate_cut_params(const tool& t,
					   const material& stock_material,
					   const pocket_name op_name) {
    double cut_depth, speed, feed;
    if (stock_material == ACETAL) {
      cut_depth = 0.2;
      speed = 3000;
      feed = 8.0;
    } else if (stock_material == ALUMINUM) {
      return calculate_cut_params_aluminum(t, op_name);
    } else if (stock_material == BRASS) {
      cut_depth = 0.1;
      speed = 16000;
      feed = 1.0;
    } else {
      DBG_ASSERT(false);
    }

    cout << "Chip Load Per Tooth = " << chip_load_per_tooth(t, feed, speed) << endl;

    return cut_move_parameters{feed, feed / t.num_flutes(), speed, cut_depth};
  }

  std::vector<toolpath>
  freeform_pocket::make_toolpaths(const material& stock_material,
				  const double safe_z,
				  const std::vector<tool>& tools) const {
    tool t = select_tool(tools);
    auto params = calculate_cut_params(t, stock_material, pocket_type());
    auto pocket_paths = toolpath_lines(t, params.cut_depth);
    return {toolpath(pocket_type(), safe_z, params.speed, params.feed, params.plunge_feed, t, pocket_paths)};
  }

  std::vector<toolpath>
  trace_pocket::make_toolpaths(const material& stock_material,
			       const double safe_z,
			       const std::vector<tool>& tools) const {
    tool t = select_tool(tools);
    auto params = calculate_cut_params(t, stock_material, pocket_type());

    auto pocket_paths = toolpath_lines(t, params.cut_depth);
    return {toolpath(pocket_type(), safe_z, params.speed, params.feed, params.plunge_feed, t, pocket_paths)};
  }
  
  std::vector<toolpath>
  face_pocket::make_toolpaths(const material& stock_material,
			      const double safe_z,
			      const std::vector<tool>& tools) const {
    tool t = select_tool(possible_tools);
    auto params = calculate_cut_params(t, stock_material, pocket_type());

    auto pocket_paths = toolpath_lines(t, params.cut_depth);
    return {toolpath(pocket_type(), safe_z, params.speed, params.feed, params.plunge_feed, t, pocket_paths)};
  }

  std::vector<toolpath>
  flat_pocket::make_toolpaths(const material& stock_material,
			      const double safe_z,
			      const std::vector<tool>& tools) const {
    if (possible_tools.size() == 0) {
      cout << "ERROR, no viable tools for pocket" << endl;
      DBG_ASSERT(possible_tools.size() > 0);
    }

    tool t = select_tool(possible_tools);
    auto params = calculate_cut_params(t, stock_material, pocket_type());

    auto pocket_paths = toolpath_lines(t, params.cut_depth);

    return {toolpath(pocket_type(), safe_z, params.speed, params.feed, params.plunge_feed, t, pocket_paths)};
  }
  
  freeform_pocket::freeform_pocket(double start_depthp,
				   const std::vector<index_t>& basep,
				   const triangular_mesh* p_mesh) :
    start_depth(start_depthp),
    base_inds(basep),
    mesh(p_mesh) {

    
    DBG_ASSERT(base_inds.size() > 0);

    auto bounds = mesh_bounds(base_inds, base_mesh());

    DBG_ASSERT(bounds.size() > 0);

    boundary = extract_boundary(bounds);
    holes = bounds;
  }


  std::pair<oriented_polygon, std::vector<oriented_polygon>>
  optimize_pocket_size(const std::vector<index_t> base,
		       const triangular_mesh& mesh) {

    DBG_ASSERT(base.size() > 0);

    auto bounds = mesh_bounds(base, mesh);

    DBG_ASSERT(bounds.size() > 0);

    auto boundary = extract_boundary(bounds);

    DBG_ASSERT(area(boundary) > 0.001);
    
    auto holes = bounds;

    return std::make_pair(boundary, holes);
  }

  flat_pocket::flat_pocket(double p_start_depth,
			   double p_end_depth,
			   const oriented_polygon& p_boundary,
			   const std::vector<tool>& p_tools) :
    boundary(p_boundary),
    holes{},
    start_depth(p_start_depth),
    end_depth(p_end_depth),
    possible_tools(p_tools) {
      //      DBG_ASSERT(area(boundary) > 0.001);
    }

  flat_pocket::flat_pocket(double p_start_depth,
			   double p_end_depth,
			   const oriented_polygon& p_boundary,
			   const std::vector<oriented_polygon>& p_holes,
			   const std::vector<tool>& p_tools) :
    boundary(p_boundary),
    holes(p_holes),
    start_depth(p_start_depth),
    end_depth(p_end_depth),
    possible_tools(p_tools) {
    //      DBG_ASSERT(area(boundary) > 0.001);

      check_simplicity(boundary);

      for (auto h : holes) {
	check_simplicity(h);
      }

    }
  
  flat_pocket::flat_pocket(double start_depthp,
			   const std::vector<index_t>& basep,
			   const triangular_mesh* p_mesh,
			   const std::vector<tool>& p_tools) :
    start_depth(start_depthp),
    end_depth(p_mesh->face_triangle(basep.front()).v1.z),
    possible_tools(p_tools) {

    pair<oriented_polygon, std::vector<oriented_polygon>> bound_and_holes =
      optimize_pocket_size(basep, *p_mesh);

    boundary = bound_and_holes.first;
    holes = bound_and_holes.second;
  }

  polygon_3 flat_pocket::base() const {
    vector<vector<point>> hs;
    for (auto h : get_holes()) {
      hs.push_back(h.vertices());
    }

    return polygon_3(get_boundary().vertices(), hs);
  }

  tool
  flat_pocket::select_tool(const std::vector<tool>& tools) const {
    vector<tool> to_check = tools;
    sort_gt(to_check, [](const tool& t) { return t.diameter(); });

    polygon_3 base_poly = base();

    for (auto& t : to_check) {
      auto offset_base_maybe = shrink_optional(base_poly, t.radius());

      if (offset_base_maybe) {
	polygon_3 offset_base = *offset_base_maybe;

	if (offset_base.holes().size() == base_poly.holes().size()) {

	  cout << "Chosen tool: " << endl;
	  cout << t << endl;
	  cout << "Cut area    = " << area(offset_base) << endl;
	  cout << "Pocket area = " << area(base_poly) << endl;

	  return t;
	}
      }
    }

    cout << "ERROR: No viable tools" << endl;
    cout << "Tools considered: " << endl;
    for (auto t : tools) {
      cout << t << endl;
    }
    
    
    vtk_debug_polygon(base_poly);

    DBG_ASSERT(false);

  }

  std::vector<polyline>
  face_level(const oriented_polygon& inter,
	     const tool& t,
	     const double cut_depth) {

    vector<polyline> polys;
    polys.push_back(to_polyline(inter));

    double r = t.radius();
    auto last_polygon = inter;
    vector<oriented_polygon> i = interior_offset(inter, r);

    while ((i.size() == 1) && contains(last_polygon, i.front())) {

      polys.push_back(to_polyline(i.front()));
      last_polygon = i.front();
      r += t.radius();
      i = interior_offset(inter, r);

    }

    return polys;
  }

  boost_linestring_2 to_boost_linestring(const polyline& pl) {
    boost_linestring_2 res;
    for (auto p : pl) {
      bg::append(res, bg::model::d2::point_xy<double>(p.x, p.y));
    }
    return res;
  }

  boost_multilinestring_2
  to_boost_multilinestring_2(const std::vector<polyline>& lines) {
    boost_multilinestring_2 res;
    for (auto pl : lines) {
      res.push_back(to_boost_linestring(pl));
    }
    return res;
  }

  boost_poly_2 to_boost_poly(const oriented_polygon& pl) {
    boost_poly_2 res;
    for (auto p : pl.vertices()) {
      bg::append(res, bg::model::d2::point_xy<double>(p.x, p.y));
    }

    bg::correct(res);

    return res;
  }

  boost_multipoly_2
  to_boost_multipoly_2(const std::vector<oriented_polygon>& lines) {
    boost_multipoly_2 res;
    for (auto& pl : lines) {
      res.push_back(to_boost_poly(pl));
    }
    return res;
  }

  boost_multipoly_2
  to_boost_multipoly_2(const std::vector<polygon_3>& lines) {
    boost_multipoly_2 res;
    for (auto& pl : lines) {
      res.push_back(to_boost_poly_2(pl));
    }
    return res;
  }

  boost_multipoly_2
  to_boost_multipoly_2(const rotation&r, const std::vector<polygon_3>& lines) {
    boost_multipoly_2 res;
    for (auto& pl : lines) {
      res.push_back(to_boost_poly_2(apply(r, pl)));
    }
    return res;
  }

  vector<polygon_3> from_boost_multipoly_2(const boost_multipoly_2& p,
					   const rotation& r,
					   const double z_level) {
    vector<polygon_3> polys;
    for (auto pl : p) {
      polys.push_back(apply(r, to_polygon_3(z_level, pl)));
    }
    return polys;
  }
  
  polyline to_polyline(const boost_linestring_2& l,
		       const double z) {
    vector<point> pts;
    for (boost_point_2 line_pt : l) {
      pts.push_back(point(line_pt.get<0>(), line_pt.get<1>(), z));
    }
    return pts;
  }
  
  std::vector<polyline> to_polylines(const boost_multilinestring_2& lines,
				     const double z) {
    std::vector<polyline> res;
    for (const boost_linestring_2& l : lines) {
      res.push_back(to_polyline(l, z));
    }
    return res;
  }

  polygon_3
  make_interior_bound(const polygon_3& bound,
		      const tool& t) {
    auto bound_p = bound;
    bound_p.correct_winding_order(point(0, 0, 1));

    // TODO: Really should use bounding box polygon instead
    polygon_3 outer_bound =
      dilate(bound_p, 10*t.radius());

    polygon_3 inner_b = shrink(bound_p, t.radius());
    polygon_3 inner_bound(outer_bound.vertices(), {inner_b.vertices()});

    return inner_bound;
  }

  polygon_3
  make_contour_bound(const polygon_3& bound,
		     const tool& t) {
    auto bound_p = bound;
    bound_p.correct_winding_order(point(0, 0, 1));

    // TODO: Really should use bounding box polygon instead
    polygon_3 outer_bound =
      dilate(bound_p, 10*t.radius());

    polygon_3 inner_bound(outer_bound.vertices(), {bound_p.vertices()});

    return inner_bound;
  }
  
  std::vector<polyline>
  clip_lines(const std::vector<polyline>& lines,
	     const polygon_3& bound,
	     const std::vector<polygon_3>& hole_polys,
	     const tool& t) {
    if (lines.size() == 0) { return lines; }

    
    double z = lines.front().front().z;
    boost_multilinestring_2 ml = to_boost_multilinestring_2(lines);
    boost_multipoly_2 hole_poly = to_boost_multipoly_2(hole_polys);

    boost_multilinestring_2 hole_result;
    bg::difference(ml, hole_poly, hole_result);


    polygon_3 bound_p = make_contour_bound(bound, t);
    boost_multipoly_2 bound_poly{to_boost_poly_2(bound_p)};

    boost_multilinestring_2 result;
    bg::difference(hole_result, bound_poly, result);

    vector<polyline> clipped = to_polylines(result, z);

    return clipped;
  }

  std::vector<polyline>
  zig_lines(const polygon_3& bound,
	    const std::vector<polygon_3>& holes,
	    const tool& t) {

    DBG_ASSERT(bound.holes().size() == 0);

    box b = bounding_box(bound);
    cout << "Zig lines bounding box = " << endl << b << endl;
    double stepover = t.radius();

    vector<polyline> lines;
    double current_y = b.y_min;
    while (current_y < b.y_max) {
      point start(b.x_min, current_y, b.z_min);
      point end(b.x_max, current_y, b.z_min);
      vector<point> pts{start, end};
      lines.push_back(pts);

      current_y += stepover;
    }

    cout << "# of lines = " << lines.size() << endl;

    lines = clip_lines(lines, bound, holes, t);

    cout << "# of lines after clipping = " << lines.size() << endl;

    return lines;
  }

  std::vector<polyline>
  flat_pocket::flat_level_with_holes(const tool& t) const {
    vector<polygon_3> offset_holes = exterior_offset(get_holes(), t.radius());

    polygon_3 inner_bound = shrink(get_boundary(), t.radius()); //make_interior_bound(get_boundary(), t);

    vector<polyline> edges = zig_lines(inner_bound, offset_holes, t);

    auto inter = interior_offset(boundary, t.radius());

    for (auto i : inter) {
      edges.push_back(to_polyline(i));
    }

    // TODO: Proper polygon merging
    for (auto h : holes) {
      auto outer = exterior_offset(h, t.radius());
      DBG_ASSERT(outer.size() == 2);
      edges.push_back(to_polyline(outer.back()));
    }

    return edges;
  }

  std::vector<polyline>
  flat_pocket::toolpath_lines(const tool& t,
			      const double cut_depth) const {
    vector<polyline> face_template;
    face_template = flat_level_with_holes(t);

    vector<double> depths =
      cut_depths(get_start_depth(), get_end_depth(), cut_depth);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(face_template, depth));
    }
    return lines;
  }

  tool
  trace_pocket::select_tool(const std::vector<tool>& tools) const {
    tool t = *(min_element(begin(tools), end(tools),
  			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));
    return t;
  }

  std::vector<polyline>
  trace_pocket::toolpath_lines(const tool& t,
			       const double cut_depth) const {

    vector<polyline> face_template{to_polyline(outline)};
    vector<double> depths =
      cut_depths(get_start_depth(), get_end_depth(), cut_depth);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(face_template, depth));
    }
    return lines;
  }

  vector<polyline> deepen_polyline(const vector<double>& depths, const polyline& p) {
    vector<polyline> ps;
    for (auto depth : depths) {
      vector<point> pts;
      for (auto pt : p)
	{ pts.push_back(point(pt.x, pt.y, depth)); }
      ps.push_back(polyline(pts));
    }
    return ps;
  }

  vector<double> cut_depths(double start_depth,
			    double end_depth,
			    double cut_depth) {
    vector<double> depths;
    double depth = max(end_depth, start_depth - cut_depth);
    while (true) {
      depths.push_back(depth);
      if (depth == end_depth) {
	break;
      }
      depth = max(end_depth, depth - cut_depth);
    }
    return depths;
  }

  vector<polyline> tile_vertical(const vector<polyline>& ps,
				 double start_depth,
				 double end_depth,
				 double cut_depth) {
    vector<polyline> vertical_slices;
    auto depths = cut_depths(start_depth, end_depth, cut_depth);
    for (auto p : ps) {
      auto cuts = deepen_polyline(depths, p);
      vertical_slices.insert(vertical_slices.end(), cuts.begin(), cuts.end());
    }
    stable_sort(vertical_slices.begin(), vertical_slices.end(),
    		[](const polyline& l, const polyline& r)
    		{ return l.front().z > r.front().z; });
    return vertical_slices;
  }

  vector<polyline> repeated_offsets(const polyline& p,
				    int num_repeats,
				    offset_dir d,
				    double inc) {
    DBG_ASSERT(num_repeats > 0);
    vector<polyline> paths;
    paths.push_back(offset(p, d, inc));
    for (int i = 1; i < num_repeats; i++) {
      paths.push_back(offset(paths.back(), d, inc));
    }
    reverse(paths.begin(), paths.end());
    return paths;
  }

  vector<polyline> finish_passes(const vector<oriented_polygon>& holes,
				 const oriented_polygon& boundary,
				 const vector<double>& depths,
				 const double tool_radius) {
    vector<polyline> lines;
    polyline p(boundary.vertices());
    lines.push_back(p);
    
    for (auto hole : holes) {
      polyline p(hole.vertices());
      lines.push_back(p);
    }
    // TODO: Change to tile vertical
    return lines;
  }

  vector<polyline> finish_pocket(const freeform_pocket& pocket,
				 const tool& t,
				 const double cut_depth) {
    auto holes = pocket.get_holes();
    vector<oriented_polygon> offset_h;//(holes.size());
    for (auto h : holes) {
      concat(offset_h, exterior_offset(h, t.radius()));
    }
    auto i_off = interior_offset(pocket.get_boundary(), t.radius());
    DBG_ASSERT(i_off.size() == 1);
    oriented_polygon bound_poly = i_off.front();
    vector<double> depths = cut_depths(pocket.get_start_depth(),
				       pocket.get_end_depth(),
				       cut_depth);
    auto ls = finish_passes(offset_h, bound_poly, depths, t.radius());
    vector<polyline> lines;
    for (auto pl : ls) {
      auto pts = drop_points_onto(vector<point>(begin(pl), end(pl)),
				  pocket.base_face_indexes(),
				  pocket.base_mesh(),
				  t);
      if (pts.size() > 0) {
	lines.push_back(pts);
      }
    }
    return lines;
  }

  vector<polyline> roughing_lines(const freeform_pocket& p,
				  const vector<triangle>& base,
				  const vector<oriented_polygon>& holes,
  				  const oriented_polygon& boundary,
  				  double last_level,
				  const tool& t) {
    double sample_increment = t.radius();

    vector<polyline> ls =
      sample_lines_2d(boundary, sample_increment, sample_increment, last_level);

    vector<polyline> lines;
    for (auto l : ls) {
      vector<point> toolpath_points =
	drop_points_onto_max(vector<point>(begin(l), end(l)),
			     p.base_face_indexes(),
			     p.base_mesh(),
			     last_level,
			     t);
      if (toolpath_points.size() > 1) {
	lines.push_back(toolpath_points);
      }
    }

    return lines;
  }

  vector<polyline> finish_base_lines(const freeform_pocket& p,
				     const tool& t,
				     const double cut_depth) {
    auto holes = p.get_holes();
    vector<oriented_polygon> offset_h;
    for (auto h : holes) {
      concat(offset_h, exterior_offset(h, t.radius()));
    }
    
    auto i_off = interior_offset(p.get_boundary(), t.radius());
    DBG_ASSERT(i_off.size() == 1);
    oriented_polygon bound_poly = i_off.front();

    vector<polyline> plines = roughing_lines(p,
					     p.base(),
					     offset_h,
					     bound_poly,
					     p.get_end_depth(),
					     t);

    return plines;
  }

  vector<polyline> roughing_passes(const freeform_pocket& p,
				   const vector<oriented_polygon>& holes,
				   const oriented_polygon& boundary,
				   const vector<double>& depths,
				   const tool& t) {
    vector<polyline> lines;
    for (auto depth : depths) {
      auto rough_level = roughing_lines(p, p.base(), holes, boundary, depth, t);
      concat(lines, rough_level);
    }
    return lines;
  }

  vector<polyline> rough_pocket(const freeform_pocket& pocket,
				const tool& t,
				double cut_depth) {
    auto holes = pocket.get_holes();
    vector<oriented_polygon> offset_h;
    for (auto h : holes) {
      concat(offset_h, exterior_offset(h, t.radius()));
    }
    
    auto i_off = interior_offset(pocket.get_boundary(), t.radius());
    cout << "# of offset = " << i_off.size() << endl;

    if (i_off.size() != 1) { return {}; }
    oriented_polygon bound_poly = i_off.front();

    vector<double> depths = cut_depths(pocket.get_start_depth(),
				       pocket.get_end_depth(),
				       cut_depth / 2.0);
    // Leave the final level to the finishing pass
    depths.pop_back();
    vector<polyline> pocket_path = roughing_passes(pocket,
						   offset_h,
						   bound_poly,
						   depths,
						   t);
    return pocket_path;
  }

  // TODO: Use tile vertical?
  std::vector<polyline>
  face_pocket::toolpath_lines(const tool& t,
			      const double cut_depth) const {
    auto inter = project(base, get_end_depth());
    vector<polyline> face_template =
      face_level(inter, t, cut_depth);

    vector<double> depths =
      cut_depths(get_start_depth(), get_end_depth(), cut_depth);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(face_template, depth));
    }

    return lines;
  }

  tool
  face_pocket::select_tool(const std::vector<tool>& tools) const {
    tool t = *(max_element(begin(tools), end(tools),
  			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));
    return t;
  }

  tool
  freeform_pocket::select_tool(const std::vector<tool>& tools) const {
    tool t = *(min_element(begin(tools), end(tools),
  			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));
    return t;
  }

  // TODO: Reintroduce this mesh cutting strategy when it is better
  // supported
  std::vector<polyline>
  freeform_pocket::toolpath_lines(const tool& t,
				  const double cut_depth) const {
    // vector<polyline> pocket_path = rough_pocket(*this, t, cut_depth);
    // auto finish_surface = finish_base_lines(*this, t, cut_depth);
    // concat(pocket_path, finish_surface);
    // auto finish_edges = finish_pocket(*this, t, cut_depth);
    // concat(pocket_path, finish_edges);
    // return pocket_path;
    return {to_polyline(project(boundary, get_start_depth()))};
  }

  // TODO: Move these to somewhere else, they really dont belong here
  
  bool same_slope(const line l, const line r, double t) {
    return within_eps(angle_between(l.end - l.start, r.end - r.start), 0, t);
  }

  polyline compress_lines(const polyline& p, double tolerance) {
    DBG_ASSERT(p.num_points() > 1);
    if (p.num_points() == 2) { return p; }
    vector<vector<line>> slope_groups;
    split_by(p.lines(), slope_groups,
	     [tolerance](const line l, const line r)
	     { return same_slope(l, r, tolerance); });
    vector<point> pts;
    for (auto slope_group : slope_groups) {
      pts.push_back(slope_group.front().start);
      pts.push_back(slope_group.back().end);
    }
    return polyline(pts);
  }

  // TODO: Actually compensate for tool radius and shape
  std::vector<point> drop_points_onto(const std::vector<point>& pts_z,
				      const std::vector<index_t>& faces,
				      const triangular_mesh& mesh,
				      const tool& tool) {
    vector<point> pts;
    for (auto pt : pts_z) {
      maybe<double> za = z_at(pt.x, pt.y, faces, mesh);
      if (za.just) {
	pts.push_back(point(pt.x, pt.y, za.t));
      }
    }
    return pts;
  }

  // TODO: Compensate for tool radius / shape and come up with a
  // more descriptive name
  std::vector<point> drop_points_onto_max(const std::vector<point>& pts_z,
					  const std::vector<index_t>& faces,
					  const triangular_mesh& mesh,
					  const double max,
					  const tool& tool) {
    vector<point> pts;
    for (auto pt : pts_z) {
      maybe<double> za = z_at(pt.x, pt.y, faces, mesh);
      if (za.just) {
	if (za.t > max) {
	  pts.push_back(point(pt.x, pt.y, za.t));
	} else {
	  pts.push_back(point(pt.x, pt.y, max));
	}
      } else {
	maybe<double> zb = mesh.z_at(pt.x, pt.y);
	if (zb.just) {
	  if (zb.t > max) {
	    pts.push_back(point(pt.x, pt.y, zb.t));
	  } else {
	    pts.push_back(point(pt.x, pt.y, max));
	  }
	} else {
	  pts.push_back(point(pt.x, pt.y, max));
	}
      }
    }
    return pts;
  }
  
  std::vector<polyline> drop_sample(const triangular_mesh& mesh,
				    const tool& tool) {
    box b = mesh.bounding_box();
    b.x_min += 0.01;
    b.y_min += 0.01;
    b.z_min += 0.01;

    vector<point> pts_z = sample_points_2d(b, tool.radius(), tool.radius(), 1.0);

    vector<point> pts = drop_points_onto(pts_z, mesh.face_indexes(), mesh, tool);

    vector<polyline> lines;
    lines.push_back(pts);
    return shift_lines(lines, point(0, 0, tool.length()));
  }

  // TODO: Does pocket list need to be non-const?
  vector<toolpath> mill_pockets(vector<pocket>& pockets,
				const std::vector<tool>& tools,
				const material& stock_material) {
    DBG_ASSERT(pockets.size() > 0);

    double h = (*(max_element(begin(pockets), end(pockets),
			      [](const pocket& l, const pocket& r)
      { return l.get_start_depth() < r.get_start_depth(); }))).get_start_depth();

    double clearance = 0.5;
    double safe_z = h + clearance;
    
    vector<toolpath> toolpaths;
    for (auto pocket : pockets) {
      vector<toolpath> tps = pocket.make_toolpaths(stock_material, safe_z, tools);
      concat(toolpaths, tps);
    }

    return toolpaths;
  }

  std::vector<polyline>
  contour::flat_level_with_holes(const tool& t) const {

    vector<polygon_3> offset_holes = exterior_offset(get_holes(), t.radius());
    vector<polyline> edges = zig_lines(get_boundary(), offset_holes, t);

    for (auto& offset_hole : offset_holes) {
      edges.push_back(to_polyline(oriented_polygon(point(0, 0, 1), offset_hole.vertices())));
      for (auto& hole_in_offset_hole : offset_hole.holes()) {
	edges.push_back(to_polyline(oriented_polygon(point(0, 0, 1), hole_in_offset_hole)));
      }
    }

    return edges;
  }

  tool contour::select_tool(const std::vector<tool>& tools) const {
    vector<tool> to_check = tools;
    sort_gt(to_check, [](const tool& t) { return t.diameter(); });

    polygon_3 base_poly = base();

    if (base_poly.holes().size() == 0) {
      return to_check.front();
    }

    vector<polygon_3> hole_polys;
    for (auto h : base_poly.holes()) {
      hole_polys.push_back(polygon_3(h));
    }

    for (auto& t : to_check) {

      vector<polygon_3> offset_holes = exterior_offset(hole_polys, t.radius());

      if (offset_holes.size() == hole_polys.size()) {

	cout << "Chosen tool: " << endl;
	cout << t << endl;
	cout << "Pocket area = " << area(base_poly) << endl;

	return t;
      }
    }

    cout << "ERROR: No viable tools" << endl;
    cout << "Tools considered: " << endl;
    for (auto t : tools) {
      cout << t << endl;
    }
    
    vtk_debug_polygon(base_poly);

    DBG_ASSERT(false);
  }

  std::vector<polyline>
  contour::toolpath_lines(const tool& t, const double cut_depth) const {
    vector<polyline> face_template;

    if (get_holes().size() == 0) {
      auto inter = project(get_boundary(), get_end_depth());

      face_template =
	face_level(oriented_polygon(point(0, 0, 1), inter.vertices()), t, cut_depth);
    } else {
      face_template = flat_level_with_holes(t);
    }

    vector<double> depths =
      cut_depths(get_start_depth(), get_end_depth(), cut_depth);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(face_template, depth));
    }
    return lines;
  }

  std::vector<polygon_3>
  contour_remaining_area(const polygon_3& bound,
			 const std::vector<polygon_3>& holes,
			 const tool& t) {
    DBG_ASSERT(false);
  }
  
  std::vector<polygon_3>
  contour_access_area(const polygon_3& bound,
		      const std::vector<polygon_3>& holes,
		      const tool& t) {
    DBG_ASSERT(false);
  }

  std::vector<polygon_3>
  polygon_intersection(const std::vector<polygon_3>& as,
		       const std::vector<polygon_3>& bs) {
    if (as.size() == 0 || bs.size() == 0) { return {}; }

    const rotation r = rotate_from_to(as.front().normal(), point(0, 0, 1));
    const rotation r_inv = inverse(r);
    double z_level = as.front().vertices().front().z;

    boost_multipoly_2 ap = to_boost_multipoly_2(r, as);
    boost_multipoly_2 bp = to_boost_multipoly_2(r, bs);
    boost_multipoly_2 cp;
    bg::intersection(ap, bp, cp);

    return from_boost_multipoly_2(cp, r, z_level);
  }

  std::vector<polygon_3>
  polygon_difference(const std::vector<polygon_3>& as,
		     const std::vector<polygon_3>& bs) {
    if (as.size() == 0 || bs.size() == 0) { return {}; }

    const rotation r = rotate_from_to(as.front().normal(), point(0, 0, 1));
    const rotation r_inv = inverse(r);
    double z_level = as.front().vertices().front().z;

    boost_multipoly_2 ap = to_boost_multipoly_2(r, as);
    boost_multipoly_2 bp = to_boost_multipoly_2(r, bs);
    boost_multipoly_2 cp;
    bg::difference(ap, bp, cp);

    return from_boost_multipoly_2(cp, r, z_level);
  }
  
  toolpath contour_cleanup_path(const std::vector<polygon_3>& remaining_area,
				const material& stock_material,
				const double safe_z,
				const std::vector<tool>& tools,
				const polygon_3& boundary,
				const std::vector<polygon_3>& holes,
				const double start_depth,
				const double end_depth) {
    DBG_ASSERT(tools.size() > 0);

    tool t = min_e(tools, [](const tool& t) { return t.cut_diameter(); });
    auto params = calculate_cut_params(t, stock_material, CONTOUR);

    vector<polygon_3> accessable_area = contour_access_area(boundary, holes, t);
    vector<polygon_3> cut_area =
      polygon_intersection(accessable_area, remaining_area);

    vector<polyline> cut_paths;

    return toolpath(CONTOUR,
		    safe_z,
		    params.speed,
		    params.feed,
		    params.plunge_feed,
		    t,
		    cut_paths);
  }

  vector<polygon_3>
  accessable_regions(const polygon_3& area, const tool& t) {
    vector<polygon_3> rs = interior_offset({area}, t.radius());
    return rs;
  }

  flat_region residual_flat_region(const flat_region& r, const tool& t) {
    vector<polygon_3> access_area = accessable_regions(r.safe_area, t);
    vector<polygon_3> residual =
      polygon_difference(r.machine_area, access_area);
    return flat_region(r.safe_area, residual, r.start_depth, r.end_depth, r.stock_material);
  }

  vector<polygon_3> safe_cut_regions(const flat_region& r, const tool& t) {
    vector<polygon_3> acc_regions = accessable_regions(r.safe_area, t);

    vector<polygon_3> cut_regions =
      polygon_intersection(acc_regions, r.machine_area);

    return cut_regions;
  }

  std::vector<polyline>
  zig_lines(const flat_region& r, const tool& t, const double cut_depth) {
    vector<polygon_3> cut_regions = safe_cut_regions(r, t);

    vtk_debug_polygons(cut_regions);

    vector<polyline> face_template;
    for (auto cut_region : cut_regions) {
      vector<polygon_3> holes;
      for (auto h : cut_region.holes()) {
	holes.push_back(polygon_3(h));
      }

      vector<polyline> edges =
	zig_lines(polygon_3(cut_region.vertices()), holes, t);
      concat(face_template, edges);
    }
    
    vector<double> depths =
      cut_depths(r.start_depth, r.end_depth, cut_depth);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(face_template, depth));
    }
    return lines;
  }
  
  toolpath zig_rough_path(const flat_region& r,
			  const double safe_z,
			  const tool& t) {

    // TODO: Update operation names?
    auto params = calculate_cut_params(t, r.stock_material, CONTOUR);

    auto pocket_paths = zig_lines(r, t, params.cut_depth);

    toolpath rough_path =
      toolpath(CONTOUR,
    	       safe_z,
    	       params.speed,
    	       params.feed,
    	       params.plunge_feed,
    	       t,
    	       pocket_paths);

    return rough_path;
  }

  tool select_roughing_tool(const flat_region& r,
			    const std::vector<tool>& tools) {
    return max_e(tools, [](const tool& t) { return t.cut_diameter(); });
  }

  tool select_finishing_tool(const flat_region& r,
			     const std::vector<tool>& tools) {
    return min_e(tools, [](const tool& t) { return t.cut_diameter(); });
  }
  
  toolpath rough_flat_region(const flat_region& r,
			     const double safe_z,
			     const std::vector<tool>& tools) {
    tool t = select_roughing_tool(r, tools);
    return zig_rough_path(r, safe_z, t);
  }

  // toolpath clean_flat_region(const flat_region& r,
  // 			     const double safe_z,
  // 			     const std::vector<tool>& tools) {
  //   DBG_ASSERT(false);
  // }

  std::vector<polyline> finish_lines(const flat_region& r,
				     const tool& t) {
    vector<polyline> edges;

    for (auto& region_poly : r.machine_area) {

      vector<polygon_3> holes;
      for (auto h : region_poly.holes()) {
	holes.push_back(polygon_3(h));
      }
      
      vector<polygon_3> offset_holes = exterior_offset(holes, t.radius());

      for (auto& offset_hole : offset_holes) {
	edges.push_back(to_polyline(oriented_polygon(point(0, 0, 1), offset_hole.vertices())));
	for (auto& hole_in_offset_hole : offset_hole.holes()) {
	  edges.push_back(to_polyline(oriented_polygon(point(0, 0, 1), hole_in_offset_hole)));
	}
      }
    }

    return edges;
  }

  toolpath finish_path(const flat_region& r,
		       const double safe_z,
		       const tool& t) {
    // TODO: Special finish cut params?
    auto params = calculate_cut_params(t, r.stock_material, CONTOUR);

    auto finish_paths = finish_lines(r, t);

    toolpath finish_path =
      toolpath(CONTOUR,
    	       safe_z,
    	       params.speed,
    	       params.feed,
    	       params.plunge_feed,
    	       t,
    	       finish_paths);

    return finish_path;
  }
  
  toolpath finish_flat_region(const flat_region& r,
			      const double safe_z,
			      const std::vector<tool>& tools) {
    tool t = select_finishing_tool(r, tools);
    return finish_path(r, safe_z, t);
  }

  std::vector<toolpath>
  machine_flat_region(const flat_region& r,
		      const double safe_z,
		      const std::vector<tool>& tools) {

    DBG_ASSERT(tools.size() > 0);

    toolpath rough_path = rough_flat_region(r, safe_z, tools);

    flat_region residual_region = residual_flat_region(r, rough_path.t);

    toolpath finish_path = finish_flat_region(r, safe_z, tools);

    return {rough_path, finish_path};

  }

  std::vector<toolpath>
  contour::make_toolpaths(const material& stock_material,
			  const double safe_z,
			  const std::vector<tool>& tools) const {
    if (possible_tools.size() == 0) {
      cout << "ERROR, no viable tools for pocket" << endl;
      DBG_ASSERT(possible_tools.size() > 0);
    }

    tool max_tool = max_e(possible_tools,
			  [](const tool& t) { return t.cut_diameter(); });

    polygon_3 safe_area = exterior_offset(get_boundary(), 3*max_tool.radius());

    for (auto h : get_holes()) {
      DBG_ASSERT(h.holes().size() == 0);

      safe_area.add_hole(h.vertices());
    }

    flat_region r(safe_area, bp, get_start_depth(), get_end_depth(), stock_material);

    return machine_flat_region(r, safe_z, tools);
  }

}
