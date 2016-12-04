#include "backend/drilled_hole_operation.h"

namespace gca {

  std::vector<polyline>
  drilled_hole_operation::toolpath_lines(const tool&,
					 const double) const {
    DBG_ASSERT(t.type() == TWIST_DRILL);

    point end = loc;
    double depth = start_depth - end_depth;

    DBG_ASSERT(depth > 0.0);

    point n(0, 0, 1);
    point start = loc + depth*n;

    polyline drill{{start, end}};
    return {drill};
  }

  struct drill_feeds {
    double speed;
    double feed;
  };

  drill_feeds calculate_drill_feeds(const material& stock_material,
				    const tool& t) {
    DBG_ASSERT(t.type() == TWIST_DRILL);
    DBG_ASSERT(stock_material == ALUMINUM);

    double diam = t.cut_diameter();

    // TODO: Add real calculations for these parameters
    double target_sfm = 250;
    double inches_per_rev = 0.004;

    double spindle_speed = (target_sfm*12.0)*diam*M_PI;
    double feedrate = inches_per_rev*spindle_speed;

    cout << "Spindle speed = " << spindle_speed << endl;
    cout << "Feedrate      = " << feedrate << endl;

    return {spindle_speed, feedrate};
  }
  
  std::vector<toolpath>
  drilled_hole_operation::make_toolpaths(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>&) const {
    DBG_ASSERT(t.type() == TWIST_DRILL);

    drill_feeds fs = calculate_drill_feeds(stock_material, t);

    vector<polyline> lines = toolpath_lines(t, 0.1);

    return {toolpath(DRILLED_HOLE_POCKET,
		     safe_z,
		     fs.speed,
		     fs.feed,
		     fs.feed,
		     //		     2000,
		     //		     5.0,
		     //		     2.5,
		     t,
		     lines)};

  }

}
