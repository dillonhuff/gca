#pragma once

#include "backend/operation.h"
#include "geometry/surface.h"
#include "backend/toolpath.h"

namespace gca {

  class freeform_operation {
  protected:
    surface surf;
    std::vector<tool> tools;
    double start_depth;
    double end_depth;

  public:
    freeform_operation(const surface& surf_p,
		       const std::vector<tool>& t_p) : surf(surf_p), tools(t_p) {

      for (auto t : tools) {
	DBG_ASSERT(t.type() == BALL_NOSE);
      }

      start_depth = max_in_dir(surf, point(0, 0, 1));
      end_depth = min_in_dir(surf, point(0, 0, 1));
    }

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;

    pocket_name pocket_type() const { return FREEFORM_POCKET; }

    double get_end_depth() const { return end_depth; }
    double get_start_depth() const { return start_depth; }

    // NOTE: Not really this is irrelevant for freeform surfaces
    double volume() const { return 0.0; }

    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z,
		   const std::vector<tool>&) const;

    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z) const;
    
  };
  
  toolpath
  freeform_finish_lines(const std::vector<index_t>& inds,
			const triangular_mesh& mesh,
			const tool& t,
			const double safe_z,
			const double stepover_fraction);

  toolpath
  freeform_rough_lines(const std::vector<index_t>& inds,
		       const triangular_mesh& mesh,
		       const tool& t,
		       const double safe_z,
		       const double stepover_fraction,
		       const double depth_fraction);

}
