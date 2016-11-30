#include "backend/operation.h"
#include "geometry/surface.h"

namespace gca {

  class chamfer_operation {
  protected:
    double start_depth, end_depth;
    
  public:
    chamfer_operation(const std::vector<index_t>& surf,
		      const triangular_mesh& mesh) {
      start_depth = max_in_dir(surface(&mesh, surf), point(0, 0, 1));
      end_depth = min_in_dir(surface(&mesh, surf), point(0, 0, 1));
    }

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const {
      return {};
    }

    pocket_name pocket_type() const { return CHAMFER_POCKET; }

    double get_end_depth() const { return end_depth; }
    double get_start_depth() const { return start_depth; }

    // NOTE: Not really but for edge features volume is not relevant
    double volume() const { return 0.0; }

    // TODO: Replace this dummy with a real implementation
    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z,
		   const std::vector<tool>& tools) const {
      return {toolpath(CHAMFER_POCKET, safe_z, 2000, 5.0, 2.5, tools.front(), toolpath_lines(tools.front(), 0.1))};
    }

  };

}
