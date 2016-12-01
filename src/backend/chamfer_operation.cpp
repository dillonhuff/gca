#include "backend/chamfer_operation.h"
#include "geometry/offset.h"
#include "geometry/vtk_debug.h"

namespace gca {

  std::vector<polyline>
  chamfer_operation::toolpath_lines(const tool& t, const double cut_depth) const {
    auto bounds = mesh_bounds(surf, mesh);

    vector<vector<point> > bound_rings;
    for (auto& bound : bounds) {
      bound_rings.push_back(bound.vertices());
    }

    std::vector<polygon_3> polys = arrange_rings(bound_rings);

    DBG_ASSERT(polys.size() == 1);

    polygon_3 bound = polys.front();

    DBG_ASSERT(bound.holes().size() == 1);

    // TODO: Add this as a parameter
    point mill_direction(0, 0, 1);

    double outer_z =
      signed_distance_along(bound.vertices().front(), mill_direction);
    double inner_z =
      signed_distance_along(bound.holes()[0].front(), mill_direction);

    bool outer_ring_is_lower_chamfer_edge =
      outer_z < inner_z;

    vector<point> chamfer_ring = outer_ring_is_lower_chamfer_edge ?
		bound.vertices() : bound.holes()[0];

    polygon_3 raw_chamfer_outline =
      build_clean_polygon_3(chamfer_ring);

    double offset = 0.05;
    double first_angle = angle_between(mesh.face_orientation(surf.front()), mill_direction);
    double theta = 90 - first_angle;
    double l = offset / sin(theta);
    double vertical_shift = sqrt(l*l - offset*offset); //0.05;

    vector<polygon_3> chamfer_polys{raw_chamfer_outline};

    vector<polygon_3> chamfer_paths;
    if (outer_ring_is_lower_chamfer_edge) {
      chamfer_paths = exterior_offset({chamfer_polys}, offset);
    } else {
      chamfer_paths = interior_offset({chamfer_polys}, offset);
    }

    DBG_ASSERT(chamfer_paths.size() == 1);

    polygon_3 chamfer_path = chamfer_paths.front();

    DBG_ASSERT(chamfer_path.holes().size() == 0);

    chamfer_path = shift(-1*vertical_shift*mill_direction, chamfer_path);

    // auto acts = polygon_3_actors(chamfer_path);
    // visualize_actors(acts);
    // acts.push_back(mesh_actors(mesh));
    // visualize_actors(acts);

    return {to_polyline(chamfer_path)};
  }

  std::vector<toolpath>
  chamfer_operation::make_toolpaths(const material& stock_material,
				    const double safe_z,
				    const std::vector<tool>&) const {
    DBG_ASSERT(t.type() == CHAMFER);

    vector<polyline> lines = toolpath_lines(t, 0.1);

    return {toolpath(CHAMFER_POCKET,
		     safe_z,
		     2000,
		     5.0,
		     2.5,
		     t,
		     lines)};
  }
  
}
