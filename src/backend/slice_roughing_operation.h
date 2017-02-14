#pragma once

#include "geometry/triangular_mesh.h"
#include "backend/operation_name.h"
#include "backend/tool.h"
#include "backend/toolpath.h"

namespace gca {

  class slice_roughing_operation {
  protected:
    // TODO: Absurdly inefficient to store this as a value but it will do for now
    triangular_mesh mesh;
    plane pl;
    std::vector<tool> tools;

  public:
    slice_roughing_operation(const plane pl_p,
			     const triangular_mesh& mesh_p,
			     const std::vector<tool>& t_p) :
      pl(pl_p), mesh(mesh_p), tools(t_p) {

      if (!(angle_eps(pl.normal(), point(0, 0, 1), 0.0, 0.5))) {
	cout << "pl.normal() = " << pl.normal() << endl;
	cout << "Should be   = " << point(0, 0, 1) << endl;
	DBG_ASSERT(angle_eps(pl.normal(), point(0, 0, 1), 0.0, 0.5));
      }
    }

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;

    pocket_name pocket_type() const { return SLICE_ROUGHING_POCKET; }

    double get_end_depth() const {
      return signed_distance_along(pl.pt(), point(0, 0, 1));
    }

    double get_start_depth() const {
      return max_in_dir(mesh, point(0, 0, 1));
    }

    // NOTE: Not really but this is only used for a test I dont want anymore
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
