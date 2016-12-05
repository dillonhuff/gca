#pragma once

#include "backend/operation.h"

namespace gca {

  class drilled_hole_operation {
  protected:
    double depth, diam;
    point base_loc, up_direction;
    tool t;

  public:
    drilled_hole_operation(const double p_depth,
			   const point p_loc,
			   const point p_up_direction,
			   const tool& t_p) :
      depth(p_depth),
      diam(t_p.cut_diameter()),
      base_loc(p_loc),
      up_direction(p_up_direction),
      t(t_p) {

      DBG_ASSERT(t.type() == TWIST_DRILL);

    }

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;

    pocket_name pocket_type() const { return DRILLED_HOLE_POCKET; }

    double get_end_depth() const
    { return signed_distance_along(base_loc, up_direction); } //end_depth; }
    double get_start_depth() const
    { return signed_distance_along(base_loc + depth*up_direction, up_direction); } //start_depth; }

    // NOTE: Not really but for edge features volume is not relevant
    double volume() const { return 0.0; }

    std::vector<toolpath>
    make_toolpaths(const material& stock_material,
		   const double safe_z,
		   const std::vector<tool>&) const;

  };

}
