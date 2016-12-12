#include <opencamlib/pathdropcutter.hpp>
#include <opencamlib/batchdropcutter.hpp>
#include <opencamlib/ballcutter.hpp>

#include "backend/freeform_toolpaths.h"
#include "geometry/offset.h"
#include "geometry/vtk_debug.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

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
    // ocl::CLPoint cp(s.x, s.y, s.z);
    // ocl::CLPoint ep(e.x, e.y, e.z);
    // ocl::CLPoint lp(l.x, l.y, l.z);

    // pdc.appendPoint(cp);
    // pdc.appendPoint(ep);
    // pdc.appendPoint(lp);


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

  toolpath
  freeform_finish_lines(const std::vector<index_t>& inds,
			const triangular_mesh& mesh,
			const tool& t,
			const double safe_z,
			const double stepover_fraction) {
    auto inds_cpy = inds;
    vector<oriented_polygon> bounds = mesh_bounds(inds_cpy, mesh);
    vector<vector<point> > rings;
    for (auto& b : bounds) {
      rings.push_back(b.vertices());
    }

    vector<polygon_3> polys = arrange_rings(rings);

    vtk_debug_polygons(polys);

    DBG_ASSERT(polys.size() == 1);

    polygon_3 surface_bound = polys.front();

    DBG_ASSERT(surface_bound.holes().size() == 0);

    vector<polyline> init_lines = zig_lines(surface_bound, {}, t);
    double z_min = 0.0;
    vector<polyline> lines = drop_polylines(z_min, mesh, init_lines, t);

    return {toolpath(FREEFORM_POCKET,
		     safe_z,
		     2000,
		     5.0,
		     2.5,
		     t,
		     lines)};
    
  }
  
}
