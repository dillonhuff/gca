#include "backend/slice_roughing_operation.h"

namespace gca {

  std::vector<toolpath>
  slice_roughing_operation::make_toolpaths(const material& stock_material,
					   const double safe_z,
					   const std::vector<tool>&) const {
    DBG_ASSERT(false);
  }

  std::vector<toolpath>
  slice_roughing_operation::make_toolpaths(const material& stock_material,
					   const double safe_z) const {
    DBG_ASSERT(false);
  }
  
}
