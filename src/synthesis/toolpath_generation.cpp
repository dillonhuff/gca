#include <cmath>

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

  toolpath freeform_pocket::make_toolpath(const material& stock_material,
					  const double safe_z,
					  const std::vector<tool>& tools) const {
    double cut_depth, speed, feed;
    if (stock_material == ACETAL) {
      cut_depth = 0.2;
      speed = 3000;
      feed = 8.0;
    } else if (stock_material == ALUMINUM) {
      cut_depth = 0.1;
      speed = 3000;
      feed = 5.0;
    } else if (stock_material == BRASS) {
      cut_depth = 0.1;
      speed = 16000;
      feed = 1.0;
    } else {
      DBG_ASSERT(false);
    }
    
    tool t = select_tool(tools);
    auto pocket_paths = toolpath_lines(t, cut_depth);
    return toolpath(pocket_type(), safe_z, speed, feed, t, pocket_paths);
  }

  toolpath trace_pocket::make_toolpath(const material& stock_material,
					  const double safe_z,
					  const std::vector<tool>& tools) const {
    double cut_depth, speed, feed;
    if (stock_material == ACETAL) {
      cut_depth = 0.2;
      speed = 3000;
      feed = 8.0;
    } else if (stock_material == ALUMINUM) {
      cut_depth = 0.1;
      speed = 3000;
      feed = 5.0;
    } else if (stock_material == BRASS) {
      cut_depth = 0.1;
      speed = 16000;
      feed = 1.0;
    } else {
      DBG_ASSERT(false);
    }
    
    tool t = select_tool(tools);
    auto pocket_paths = toolpath_lines(t, cut_depth);
    return toolpath(pocket_type(), safe_z, speed, feed, t, pocket_paths);
  }
  
  toolpath contour_pocket::make_toolpath(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>& tools) const {
    double cut_depth, speed, feed;
    if (stock_material == ACETAL) {
      cut_depth = 0.2;
      speed = 3000;
      feed = 8.0;
    } else if (stock_material == ALUMINUM) {
      cut_depth = 0.1;
      speed = 3000;
      feed = 5.0;
    } else if (stock_material == BRASS) {
      cut_depth = 0.1;
      speed = 16000;
      feed = 1.0;
    } else {
      DBG_ASSERT(false);
    }
    
    tool t = select_tool(tools);
    auto pocket_paths = toolpath_lines(t, cut_depth);
    return toolpath(pocket_type(), safe_z, speed, feed, t, pocket_paths);
  }

  toolpath face_pocket::make_toolpath(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>& tools) const {
    double cut_depth, speed, feed;
    if (stock_material == ACETAL) {
      cut_depth = 0.2;
      speed = 3000;
      feed = 8.0;
    } else if (stock_material == ALUMINUM) {
      cut_depth = 0.1;
      speed = 3000;
      feed = 5.0;
    } else if (stock_material == BRASS) {
      cut_depth = 0.1;
      speed = 16000;
      feed = 1.0;
    } else {
      DBG_ASSERT(false);
    }

    tool t = select_tool(tools);
    auto pocket_paths = toolpath_lines(t, cut_depth);
    return toolpath(pocket_type(), safe_z, speed, feed, t, pocket_paths);
  }

  toolpath flat_pocket::make_toolpath(const material& stock_material,
				      const double safe_z,
				      const std::vector<tool>& tools) const {
    double cut_depth, speed, feed;
    if (stock_material == ACETAL) {
      cut_depth = 0.2;
      speed = 3000;
      feed = 8.0;
    } else if (stock_material == ALUMINUM) {
      cut_depth = 0.1;
      speed = 3000;
      feed = 5.0;
    } else if (stock_material == BRASS) {
      cut_depth = 0.1;
      speed = 16000;
      feed = 1.0;
    } else {
      DBG_ASSERT(false);
    }

    if (possible_tools.size() == 0) {
      cout << "ERROR, no viable tools for pocket" << endl;
      DBG_ASSERT(possible_tools.size() > 0);
    }

    tool t = select_tool(possible_tools);
    auto pocket_paths = toolpath_lines(t, cut_depth);
    return toolpath(pocket_type(), safe_z, speed, feed, t, pocket_paths);
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
    // surface sf(&mesh, base);
    // plane pl = surface_plane(sf).slide(0.001);
    // auto polys = mesh_cross_section(mesh, pl);
    // vector<polygon_2> p = project(polys);
    

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

  tool
  flat_pocket::select_tool(const std::vector<tool>& tools) const {
    double bound_area = area(boundary);

    vector<tool> viable =
      select(tools, [bound_area](const tool& t)
	     { return t.cross_section_area() < bound_area; });

    if (viable.size() == 0) {
      vtk_debug_polygon(boundary);
      DBG_ASSERT(viable.size() > 0);
    }

    tool t = min_e(viable, [](const tool& l) { return l.diameter(); });

    return t;
  }

  std::vector<polyline>
  face_level(const oriented_polygon& inter,
	     const tool& t,
	     const double cut_depth) {

    vector<polyline> polys;
    double r = t.radius();
    auto last_polygon = inter;
    auto i = interior_offset(inter, r);

    while ((i.size() == 1) && contains(last_polygon, i.front())) {

      polys.push_back(to_polyline(i.front()));
      last_polygon = i.front();
      r += t.radius();
      i = interior_offset(inter, r);

    }

    return polys;
  }

  std::vector<polyline>
  flat_pocket::toolpath_lines(const tool& t,
			      const double cut_depth) const {
    auto inter = project(boundary, get_end_depth());
    vector<polyline> face_template =
      face_level(inter, t, cut_depth);

    vector<double> depths =
      cut_depths(get_start_depth(), get_end_depth(), cut_depth);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(face_template, depth));
    }

    // if (lines.size() == 0) {
    //   vtk_debug_polygon(boundary);
    // }
    
    return lines;
    //  return { to_polyline(project(boundary, get_end_depth())) };
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
    // vector<oriented_polygon> inter =
    //   exterior_offset(project(outline, get_end_depth()), t.radius());

    // DBG_ASSERT(inter.size() == 2);

    vector<polyline> face_template{to_polyline(outline)};
    vector<double> depths =
      cut_depths(get_start_depth(), get_end_depth(), cut_depth);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(face_template, depth));
    }
    return lines;
    //  return { to_polyline(project(boundary, get_end_depth())) };
  }
  
  pocket box_pocket(const box b) {
    point p1(b.x_min, b.y_min, b.z_min);
    point p2(b.x_min, b.y_max, b.z_min);
    point p3(b.x_max, b.y_max, b.z_min);
    point p4(b.x_max, b.y_min, b.z_min);

    vector<point> verts{p1, p2, p3, p4};
    oriented_polygon base(point(0, 0, 1), verts);
    return face_pocket(b.z_max, b.z_min, base);
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

  std::vector<polyline>
  contour_level(const oriented_polygon& outer,
		const oriented_polygon& inter,
		const tool& t,
		const double level) {
    double r = t.radius();
    
    vector<polyline> polys;

    auto i = exterior_offset(inter, r);

    cout << "# of exterior offsets = " << i.size() << endl;
    
    DBG_ASSERT(i.size() == 2);


    while ((i.size() == 2) && !contains(i.front(), outer)) {
      polys.push_back(to_polyline(i.back()));
      r += t.radius();
      i = exterior_offset(inter, r);
    }

    if (contains(i.front(), outer)) {
      polys.push_back(to_polyline(i.back()));
    }

    reverse(begin(polys), end(polys));

    return polys;
  }

  // TODO: Check legality of tool size
  tool
  contour_pocket::select_tool(const std::vector<tool>& tools) const {
    tool t = *(min_element(begin(tools), end(tools),
  			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));
    return t;
  }

  std::vector<polyline>
  contour_pocket::toolpath_lines(const tool& t,
				 const double cut_depth) const {
    auto i_off = interior_offset(exterior, t.radius());
    DBG_ASSERT(i_off.size() == 1);
    auto o = project(i_off.front(), get_end_depth());
    auto inter = project(interior, get_end_depth());
    vector<double> depths =
      cut_depths(get_start_depth(), get_end_depth(), cut_depth);
    vector<polyline> level_template =
      contour_level(o, interior, t, get_end_depth());
    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, project_lines(level_template, depth));
    }
    return lines;
    //    return { to_polyline(project(interior, get_end_depth())) };
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

    //    return { to_polyline(project(inter, get_end_depth())) };
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

    double safe_z = h + 0.5;
    
    vector<toolpath> toolpaths;
    for (auto pocket : pockets) {
      toolpath tp = pocket.make_toolpath(stock_material, safe_z, tools);
      toolpaths.push_back(tp);
    }
    return toolpaths;
  }
  
}
