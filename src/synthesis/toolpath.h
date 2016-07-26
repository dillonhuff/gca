#ifndef GCA_TOOLPATH_H
#define GCA_TOOLPATH_H

#include <vector>

#include "geometry/polyline.h"
#include "synthesis/tool.h"

namespace gca {

  struct toolpath {
    double safe_z_before_tlc;
    double spindle_speed;
    double feedrate;

    tool t;
    std::vector<polyline> lines;

    toolpath(const double p_safe_z,
	     const double p_spindle,
	     const double p_feed,
	     const tool& p_t,
	     const std::vector<polyline>& p_lines)
      : safe_z_before_tlc(p_safe_z),
	spindle_speed(p_spindle),
	feedrate(p_feed),
	t(p_t),
	lines(p_lines) {}
  };

}

#endif
