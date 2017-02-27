#include <opencamlib/pathdropcutter.hpp>
#include <opencamlib/batchdropcutter.hpp>
#include <opencamlib/ballcutter.hpp>
#include <opencamlib/cylcutter.hpp>

#include "backend/freeform_toolpaths.h"
#include "geometry/offset.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "backend/toolpath_generation.h"

namespace gca {

  std::vector<polyline>
  zig_lines_sampled_x(const polygon_3& bound,
		      const std::vector<polygon_3>& holes,
		      const tool& t,
		      const double stepover_fraction) {

    DBG_ASSERT(bound.holes().size() == 0);
    DBG_ASSERT(stepover_fraction > 0.0);
    DBG_ASSERT(stepover_fraction <= 0.5);

    box b = bounding_box(bound);
    cout << "Zig lines bounding box = " << endl << b << endl;
    double stepover = stepover_fraction*t.diameter();

    vector<polyline> lines;
    double x_stepover = 0.01;

    double current_y = b.y_min;

    while (current_y < b.y_max) {

      double current_x = b.x_min;
      vector<point> line;
      while (current_x < b.x_max) {
	point start(current_x, current_y, b.z_min);
	line.push_back(start);
	current_x += x_stepover;
      }

      lines.push_back(line);
      current_y += stepover;
    }

    cout << "# of lines = " << lines.size() << endl;

    lines = clip_lines(lines, bound, holes, t);

    cout << "# of lines after clipping = " << lines.size() << endl;

    return lines;
  }

  std::vector<polyline>
  zig_lines_sampled_y(const polygon_3& bound,
		      const std::vector<polygon_3>& holes,
		      const tool& t,
		      const double stepover_fraction) {

    DBG_ASSERT(bound.holes().size() == 0);
    DBG_ASSERT(stepover_fraction > 0.0);
    DBG_ASSERT(stepover_fraction <= 0.5);

    box b = bounding_box(bound);
    cout << "Zig lines bounding box = " << endl << b << endl;
    double stepover = stepover_fraction*t.diameter();

    vector<polyline> lines;
    double y_stepover = 0.01;

    double current_x = b.x_min;

    while (current_x < b.x_max) {

      double current_y = b.y_min;
      vector<point> line;
      while (current_y < b.y_max) {
	point start(current_x, current_y, b.z_min);
	line.push_back(start);
	current_y += y_stepover;
      }

      lines.push_back(line);
      current_x += stepover;
    }

    cout << "# of lines = " << lines.size() << endl;

    lines = clip_lines(lines, bound, holes, t);

    cout << "# of lines after clipping = " << lines.size() << endl;

    return lines;
  }

  polyline
  drop_polyline(const double z_min,
		const std::vector<triangle>& triangles,
		const polyline& init_line,
		const tool& t) {

    ocl::STLSurf surf;
    for (auto t : triangles) {
      surf.addTriangle(ocl::Triangle(ocl::Point(t.v1.x, t.v1.y, t.v1.z),
				     ocl::Point(t.v2.x, t.v2.y, t.v2.z),
				     ocl::Point(t.v3.x, t.v3.y, t.v3.z)));

    }

    ocl::BatchDropCutter pdc;
    for (auto& p : init_line) {
      ocl::CLPoint cl_pt(p.x, p.y, z_min);
      pdc.appendPoint(cl_pt);
    }

    pdc.setSTL(surf);

    if (t.type() == BALL_NOSE) {
      ocl::BallCutter ballCut(t.cut_diameter(), t.cut_length());
      pdc.setCutter(&ballCut);
    } else if (t.type() == FLAT_NOSE) {
      ocl::CylCutter millCut(t.cut_diameter(), t.cut_length());
      pdc.setCutter(&millCut);
    }
  
    pdc.run();

    vector<point> final_pts;

    auto pts = pdc.getCLPoints();
    std::cout << "# of clpoints = " << pts.size() << std::endl;
    //    for (auto pt : pts) {
    for (unsigned i = 0; i < pts.size(); i++) {
      auto pt = pts[i];
      final_pts.push_back(point(pt.x, pt.y, pt.z));
    }
    
    return final_pts;
  }
  
  polyline
  drop_polyline(const double z_min,
		const triangular_mesh& mesh,
		const polyline& init_line,
		const tool& t) {
    
    return drop_polyline(z_min, mesh.triangle_list(), init_line, t);
  }
  
  std::vector<polyline>
  drop_polylines(const double z_min,
		 const triangular_mesh& mesh,
		 const vector<polyline>& init_lines,
		 const tool& t) {

    ocl::STLSurf surf;
    for (auto t : mesh.triangle_list()) {
      surf.addTriangle(ocl::Triangle(ocl::Point(t.v1.x, t.v1.y, t.v1.z),
				     ocl::Point(t.v2.x, t.v2.y, t.v2.z),
				     ocl::Point(t.v3.x, t.v3.y, t.v3.z)));

    }

    ocl::BatchDropCutter pdc;
    for (auto& init_line : init_lines) {
      for (auto& p : init_line) {
	ocl::CLPoint cl_pt(p.x, p.y, z_min);
	pdc.appendPoint(cl_pt);
      }
    }

    pdc.setSTL(surf);

    if (t.type() == BALL_NOSE) {
      ocl::BallCutter ballCut(t.cut_diameter(), t.cut_length());
      pdc.setCutter(&ballCut);
    } else if (t.type() == FLAT_NOSE) {
      ocl::CylCutter millCut(t.cut_diameter(), t.cut_length());
      pdc.setCutter(&millCut);
    }
  
    pdc.run();

    //vector<point> final_pts;
    vector<polyline> dropped;

    auto pts = pdc.getCLPoints();
    std::cout << "# of clpoints = " << pts.size() << std::endl;

    unsigned total = 0;
    for (unsigned i = 0; i < init_lines.size(); i++) {
      vector<point> final_pts;

      for (unsigned j = 0; j < init_lines[i].num_points(); j++) {

	auto pt = pts[total];
	final_pts.push_back(point(pt.x, pt.y, pt.z));

	total++;
      }

      dropped.push_back(polyline(final_pts));

    }
    
    return dropped;

    // vector<polyline> dropped;
    // for (auto& l : init_lines) {
    //   dropped.push_back(drop_polyline(z_min, mesh, l, t));
    // }

    // return dropped;
  }

  vector<polyline>
  freeform_zig(const std::vector<index_t>& inds,
	       const triangular_mesh& mesh,
	       const tool& t,
	       const double safe_z,
	       const double z_min,
	       const double stepover_fraction) {

    //vtk_debug_highlight_inds(inds, mesh);

    // auto inds_cpy = inds;
    // vector<oriented_polygon> bounds = mesh_bounds(inds_cpy, mesh);
    // vector<vector<point> > rings;
    // for (auto& b : bounds) {
    //   rings.push_back(b.vertices());
    // }

    vector<polygon_3> polys =
      surface_boundary_polygons(inds, mesh); //arrange_rings(rings);

    //vtk_debug_polygons(polys);

    DBG_ASSERT(polys.size() == 1);

    polygon_3 surface_bound = polys.front();

    DBG_ASSERT(surface_bound.holes().size() == 0);

    
    vector<polyline> init_lines =
      zig_lines_sampled_y(surface_bound, {}, t, stepover_fraction);
    vector<polyline> lines = drop_polylines(z_min, mesh, init_lines, t);

    return lines;
  }

  
  polygon_3 box_bound(const triangular_mesh& mesh) {
    box bb = mesh.bounding_box();

    point p1(bb.x_min, bb.y_min, bb.z_min);
    point p2(bb.x_max, bb.y_min, bb.z_min);
    point p3(bb.x_max, bb.y_max, bb.z_min);
    point p4(bb.x_min, bb.y_max, bb.z_min);

    vector<point> pts{p1, p2, p3, p4};
    polygon_3 poly = build_clean_polygon_3(pts);
    poly.correct_winding_order(point(0, 0, 1));

    return poly;
  }

  vector<polyline>
  freeform_zig(const triangular_mesh& mesh,
	       const tool& t,
	       const double safe_z,
	       const double z_min,
	       const double stepover_fraction) {

    //vtk_debug_highlight_inds(inds, mesh);

    // auto inds_cpy = inds;
    // vector<oriented_polygon> bounds = mesh_bounds(inds_cpy, mesh);
    // vector<vector<point> > rings;
    // for (auto& b : bounds) {
    //   rings.push_back(b.vertices());
    // }

    // vector<polygon_3> polys =
    //   surface_boundary_polygons(inds, mesh); //arrange_rings(rings);

    // //vtk_debug_polygons(polys);

    // DBG_ASSERT(polys.size() == 1);

    // polygon_3 surface_bound = polys.front();

    // DBG_ASSERT(surface_bound.holes().size() == 0);

    polygon_3 surface_bound = box_bound(mesh);
    
    vector<polyline> init_lines =
      zig_lines_sampled_y(surface_bound, {}, t, stepover_fraction);
    vector<polyline> lines = drop_polylines(z_min, mesh, init_lines, t);

    return lines;
  }
  
  toolpath
  freeform_finish_lines(const std::vector<index_t>& inds,
			const triangular_mesh& mesh,
			const tool& t,
			const double safe_z,
			const double stepover_fraction) {
    double z_min = min_in_dir(surface(&mesh, inds), point(0, 0, 1));

    vector<polyline> lines =
      freeform_zig(inds, mesh, t, safe_z, z_min, stepover_fraction);

    return {toolpath(FREEFORM_POCKET,
		     safe_z,
		     2000,
		     15.0,
		     7.5,
		     t,
		     lines)};
  }

  toolpath
  freeform_rough_lines(const std::vector<index_t>& inds,
		       const triangular_mesh& mesh,
		       const tool& t,
		       const double safe_z,
		       const double stepover_fraction,
		       const double depth_fraction) {

    DBG_ASSERT(stepover_fraction > 0.0);
    DBG_ASSERT(stepover_fraction <= 0.5);

    surface surf(&mesh, inds);
    point up(0, 0, 1);

    double start_depth = max_in_dir(surf, up);
    double end_depth = min_in_dir(surf, up);


    cout << "Start depth = " << start_depth << endl;
    cout << "End depth   = " << end_depth << endl;

    double cut_depth = depth_fraction*t.diameter();
    vector<double> depths = cut_depths(start_depth, end_depth, cut_depth);

    cout << "# of depths = " << depths.size() << endl;
    for (auto d : depths) {
      cout << d << endl;
    }

    //vtk_debug_highlight_inds(surf);
    
    DBG_ASSERT(depths.size() > 0);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, freeform_zig(inds, mesh, t, safe_z, depth, stepover_fraction));
    }

    return {toolpath(FREEFORM_POCKET,
		     safe_z,
		     2000,
		     15.0,
		     7.5,
		     t,
		     lines)};
  }

  std::vector<polyline>
  freeform_operation::toolpath_lines(const tool& t, const double cut_depth) const {
    DBG_ASSERT(false);
  }

  std::vector<toolpath>
  freeform_operation::make_toolpaths(const material& stock_material,
				     const double safe_z,
				     const std::vector<tool>&) const {
    DBG_ASSERT(tools.size() > 0);

    tool t = min_e(tools, [](const tool& t) { return t.cut_diameter(); });

    double rough_stepover_fraction = 0.25;
    double rough_depth_fraction = 0.1;
    toolpath rough_tp =
      freeform_rough_lines(surf.index_list(),
			   surf.get_parent_mesh(),
			   t,
			   safe_z,
			   rough_stepover_fraction,
			   rough_depth_fraction);

    double stepover_fraction = 0.01;
    toolpath finish_tp =
      freeform_finish_lines(surf.index_list(),
			    surf.get_parent_mesh(),
			    t,
			    safe_z,
			    stepover_fraction);

    return {rough_tp, finish_tp};
  }

  std::vector<toolpath>
  freeform_operation::make_toolpaths(const material& stock_material,
				     const double safe_z) const {
    DBG_ASSERT(tools.size() > 0);

    tool t = min_e(tools, [](const tool& t) { return t.cut_diameter(); });

    double rough_stepover_fraction = 0.25;
    double rough_depth_fraction = 0.1;
    toolpath rough_tp =
      freeform_rough_lines(surf.index_list(),
			   surf.get_parent_mesh(),
			   t,
			   safe_z,
			   rough_stepover_fraction,
			   rough_depth_fraction);

    double stepover_fraction = 0.01;
    toolpath finish_tp =
      freeform_finish_lines(surf.index_list(),
			    surf.get_parent_mesh(),
			    t,
			    safe_z,
			    stepover_fraction);

    return {rough_tp, finish_tp};
  }
  

}
