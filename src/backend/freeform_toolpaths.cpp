#include <opencamlib/pathdropcutter.hpp>
#include <opencamlib/batchdropcutter.hpp>
#include <opencamlib/ballcutter.hpp>

#include "backend/freeform_toolpaths.h"
#include "geometry/offset.h"
#include "geometry/vtk_debug.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  std::vector<polyline>
  zig_lines_sampled(const polygon_3& bound,
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
  
  polyline
  drop_polyline(const double z_min,
		const triangular_mesh& mesh,
		const polyline& init_line,
		const tool& t) {
    DBG_ASSERT(t.type() == BALL_NOSE);

    ocl::STLSurf surf;
    for (auto t : mesh.triangle_list()) {
      surf.addTriangle(ocl::Triangle(ocl::Point(t.v1.x, t.v1.y, t.v1.z),
				     ocl::Point(t.v2.x, t.v2.y, t.v2.z),
				     ocl::Point(t.v3.x, t.v3.y, t.v3.z)));

    }

    ocl::BallCutter ballCut(t.cut_diameter(), t.cut_length());

    ocl::BatchDropCutter pdc;
    for (auto& p : init_line) {
      ocl::CLPoint cl_pt(p.x, p.y, z_min);
      pdc.appendPoint(cl_pt);
    }

    pdc.setSTL(surf);
    pdc.setCutter(&ballCut);
  
    std::vector<ocl::CLPoint> pts = pdc.getCLPoints();
    std::cout << "# of clpoints = " << pts.size() << std::endl;
    for (auto pt : pts) {
      std::cout << pt << std::endl;
    }

    pdc.run();

    vector<point> final_pts;

    pts = pdc.getCLPoints();
    std::cout << "# of clpoints = " << pts.size() << std::endl;
    for (auto pt : pts) {
      std::cout << pt << std::endl;
      final_pts.push_back(point(pt.x, pt.y, pt.z));
    }
    
    return final_pts;
  }
  
  std::vector<polyline>
  drop_polylines(const double z_min,
		 const triangular_mesh& mesh,
		 const vector<polyline>& init_lines,
		 const tool& t) {
    DBG_ASSERT(t.type() == BALL_NOSE);

    vector<polyline> dropped;
    for (auto& l : init_lines) {
      dropped.push_back(drop_polyline(z_min, mesh, l, t));
    }

    return dropped;
  }

  vector<polyline>
  freeform_zig(const std::vector<index_t>& inds,
	       const triangular_mesh& mesh,
	       const tool& t,
	       const double safe_z,
	       const double z_min,
	       const double stepover_fraction) {

    auto inds_cpy = inds;
    vector<oriented_polygon> bounds = mesh_bounds(inds_cpy, mesh);
    vector<vector<point> > rings;
    for (auto& b : bounds) {
      rings.push_back(b.vertices());
    }

    vector<polygon_3> polys = arrange_rings(rings);

    //vtk_debug_polygons(polys);

    DBG_ASSERT(polys.size() == 1);

    polygon_3 surface_bound = polys.front();

    DBG_ASSERT(surface_bound.holes().size() == 0);

    vector<polyline> init_lines =
      zig_lines_sampled(surface_bound, {}, t, stepover_fraction);
    vector<polyline> lines = drop_polylines(z_min, mesh, init_lines, t);

    return lines;
  }

  toolpath
  freeform_finish_lines(const std::vector<index_t>& inds,
			const triangular_mesh& mesh,
			const tool& t,
			const double safe_z,
			const double stepover_fraction) {
    double z_min = 0.0;

    vector<polyline> lines =
      freeform_zig(inds, mesh, t, safe_z, z_min, stepover_fraction);

    return {toolpath(FREEFORM_POCKET,
		     safe_z,
		     2000,
		     5.0,
		     2.5,
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

    vtk_debug_highlight_inds(surf);
    
    DBG_ASSERT(depths.size() > 0);

    vector<polyline> lines;
    for (auto depth : depths) {
      concat(lines, freeform_zig(inds, mesh, t, safe_z, depth, stepover_fraction));
    }

    return {toolpath(FREEFORM_POCKET,
		     safe_z,
		     2000,
		     5.0,
		     2.5,
		     t,
		     lines)};
  }

}
