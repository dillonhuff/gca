#ifndef GCA_TOOLPATH_H
#define GCA_TOOLPATH_H

#include <vector>

#include "gcode/cut.h"
#include "geometry/polyline.h"
#include "synthesis/operation_name.h"
#include "synthesis/tool.h"

namespace gca {

  class toolpath {
  public:
    pocket_name pocket_tp;
    double safe_z_before_tlc;
    double spindle_speed;
    double feedrate;
    double plunge_feedrate;

    tool t;

    std::vector<cut*> cuts;

  protected:
    std::vector<polyline> ls;

  public:
    toolpath(const pocket_name& p_pocket_type,
	     const double p_safe_z,
	     const double p_spindle,
	     const double p_feed,
	     const double p_plunge_feed,
	     const tool& p_t,
	     const std::vector<polyline>& p_lines);

    pocket_name pocket_type() const { return pocket_tp; }

    int tool_number() const { return t.tool_number(); }

    point start_location() const {
      DBG_ASSERT(this->lines().size() > 0);

      point start_pt = cuts.front()->get_start(); //lines.front().front();
      return point(start_pt.x, start_pt.y, safe_z_before_tlc);
    }

    point end_location() const {
      DBG_ASSERT(this->lines().size() > 0);

      point end_pt = cuts.back()->get_end(); //lines.front().front();
      return point(end_pt.x, end_pt.y, safe_z_before_tlc);
    }

    std::vector<polyline> lines() const { return ls; }

  };

  double execution_time_seconds(const toolpath& tp, const double rapid_feed);
  double air_time_seconds(const toolpath& tp,
			  const double rapid_feed);


}

#endif
