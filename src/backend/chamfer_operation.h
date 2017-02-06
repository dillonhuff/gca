#pragma once

#include "backend/operation.h"
#include "geometry/surface.h"

namespace gca {

  class chamfer_operation {
  protected:
    std::vector<index_t> surf;

    // TODO: Absurdly inefficient to store this as a value but it will do for now
    triangular_mesh mesh;

    double start_depth, end_depth;
    tool t;

  public:
    chamfer_operation(const std::vector<index_t>& surf_p,
		      const triangular_mesh& mesh_p,
		      const tool& t_p) : surf(surf_p), mesh(mesh_p), t(t_p) {

      DBG_ASSERT(t.type() == CHAMFER);

      start_depth = max_in_dir(surface(&mesh, surf), point(0, 0, 1));
      end_depth = min_in_dir(surface(&mesh, surf), point(0, 0, 1));
    }

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;

    pocket_name pocket_type() const { return CHAMFER_POCKET; }

    double get_end_depth() const { return end_depth; }
    double get_start_depth() const { return start_depth; }

    // NOTE: Not really but for edge features volume is not relevant
    double volume() const { return 0.0; }

    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z,
		   const std::vector<tool>&) const;

    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z) const;
    
  };

}
