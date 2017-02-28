#include "backend/slice_roughing_operation.h"

#include "backend/freeform_toolpaths.h"
#include "geometry/triangular_mesh_utils.h"

namespace gca {

  std::vector<toolpath>
  slice_roughing_operation::make_toolpaths(const material& stock_material,
					   const double safe_z,
					   const std::vector<tool>&) const {
    DBG_ASSERT(false);
  }

  tool largest_flat_nose(const std::vector<tool>& tools) {
    boost::optional<tool> max_tool = boost::none;
    for (auto& t : tools) {
      if ((t.type() == FLAT_NOSE)) {
	if (!max_tool) {
	  max_tool = t;
	} else if (max_tool->cut_diameter() < t.cut_diameter()){
	  max_tool = t;
	}
      }
    }

    DBG_ASSERT(max_tool);

    return *max_tool;
  }

  std::vector<toolpath>
  slice_roughing_operation::make_toolpaths(const material& stock_material,
					   const double safe_z) const {
    double z_min = get_end_depth();

    tool t = largest_flat_nose(tools);

    double stepover_fraction = 0.25;

    polygon_3 surface_bound = box_bound(workpiece);

    DBG_ASSERT(surface_bound.holes().size() == 0);
    
    vector<polyline> lines =
      freeform_zig(surface_bound, mesh, t, safe_z, z_min, stepover_fraction);

    return {toolpath(FREEFORM_POCKET,
		     safe_z,
		     2000,
		     15.0,
		     7.5,
		     t,
		     lines)};
    
  }
  
}
