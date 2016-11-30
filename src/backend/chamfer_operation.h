#include "backend/operation.h"

namespace gca {

  class chamfer_operation {
  public:
    chamfer_operation(const std::vector<index_t>& surf,
		      const triangular_mesh& mesh) {}

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const {
      return {};
    }

    pocket_name pocket_type() const { return CHAMFER_POCKET; }

    double get_end_depth() const { DBG_ASSERT(false); }
    double get_start_depth() const { DBG_ASSERT(false); }

    double volume() const { DBG_ASSERT(false); }

    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z,
		   const std::vector<tool>& tools) const
    { DBG_ASSERT(false); }
    
  };

}
