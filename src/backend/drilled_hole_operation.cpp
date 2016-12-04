#include "backend/drilled_hole_operation.h"

namespace gca {

  std::vector<polyline>
  drilled_hole_operation::toolpath_lines(const tool&,
					 const double cut_depth) const {
    DBG_ASSERT(t.type() == TWIST_DRILL);

    point end = loc;
    double depth = start_depth - end_depth;

    DBG_ASSERT(depth > 0.0);

    point n(0, 0, 1);
    point start = loc + depth*n;

    polyline drill{{start, end}};
    return {drill};
  }
  
  std::vector<toolpath>
  drilled_hole_operation::make_toolpaths(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>&) const {
    DBG_ASSERT(t.type() == TWIST_DRILL);

    vector<polyline> lines = toolpath_lines(t, 0.1);

    return {toolpath(DRILLED_HOLE_POCKET,
		     safe_z,
		     2000,
		     5.0,
		     2.5,
		     t,
		     lines)};

  }

}
