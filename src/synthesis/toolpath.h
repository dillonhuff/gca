#ifndef GCA_TOOLPATH_H
#define GCA_TOOLPATH_H

#include <vector>

#include "geometry/polyline.h"
#include "synthesis/operation_name.h"
#include "synthesis/tool.h"

namespace gca {

  class toolpath {
  protected:
    std::vector<polyline> lines;

  public:
    pocket_name pocket_tp;
    double safe_z_before_tlc;
    double spindle_speed;
    double feedrate;
    double plunge_feedrate;

    tool t;

    toolpath(const pocket_name& p_pocket_type,
	     const double p_safe_z,
	     const double p_spindle,
	     const double p_feed,
	     const double p_plunge_feed,
	     const tool& p_t,
	     const std::vector<polyline>& p_lines)
      : pocket_tp(p_pocket_type),
	safe_z_before_tlc(p_safe_z),
	spindle_speed(p_spindle),
	feedrate(p_feed),
	plunge_feedrate(p_plunge_feed),
	t(p_t),
	lines(p_lines) {}

    pocket_name pocket_type() const { return pocket_tp; }

    int tool_number() const { return t.tool_number(); }

    point start_location() const {
      DBG_ASSERT(lines.size() > 0);
      point start_pt = lines.front().front();
      return point(start_pt.x, start_pt.y, safe_z_before_tlc);
    }

    point end_location() const {
      DBG_ASSERT(lines.size() > 0);
      point start_pt = lines.front().front();
      return point(start_pt.x, start_pt.y, safe_z_before_tlc);
    }

  };

  double execution_time_seconds(const toolpath& tp, const double rapid_feed);
  double air_time_seconds(const toolpath& tp,
			  const double rapid_feed);


}

#endif
