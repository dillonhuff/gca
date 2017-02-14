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
      pl(pl_p), mesh(mesh_p), tools(t_p) {}

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;

    pocket_name pocket_type() const { return SLICE_ROUGHING_POCKET; }

    double get_end_depth() const { DBG_ASSERT(false); }
    double get_start_depth() const { DBG_ASSERT(false); }

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
