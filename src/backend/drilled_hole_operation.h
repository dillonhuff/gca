#include "backend/operation.h"

namespace gca {

  class drilled_hole_operation {
  protected:
    double start_depth, end_depth;
    point loc;
    tool t;

  public:
    drilled_hole_operation(const double p_start_depth,
			   const double p_end_depth,
			   const point p_loc,
			   const tool& t_p) :
      start_depth(p_start_depth),
      end_depth(p_end_depth),
      loc(p_loc),
      t(t_p) {

      DBG_ASSERT(t.type() == TWIST_DRILL);

    }

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;

    pocket_name pocket_type() const { return DRILLED_HOLE_POCKET; }

    double get_end_depth() const { return end_depth; }
    double get_start_depth() const { return start_depth; }

    // NOTE: Not really but for edge features volume is not relevant
    double volume() const { return 0.0; }

    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z,
		   const std::vector<tool>&) const;

  };

}
