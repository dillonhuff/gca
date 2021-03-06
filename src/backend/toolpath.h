#ifndef GCA_TOOLPATH_H
#define GCA_TOOLPATH_H

#include <vector>

#include "backend/operation_name.h"
#include "gcode/cut.h"
#include "geometry/polyline.h"
#include "backend/cut_params.h"
#include "backend/tool.h"

namespace gca {

  class toolpath {
  public:
    pocket_name pocket_tp;
    double safe_z_before_tlc;
    double spindle_speed;
    double feedrate;
    double plunge_feedrate;

    tool t;

  protected:
    std::vector<std::vector<cut*>> cuts;

  public:
    toolpath(const pocket_name& p_pocket_type,
	     const double p_safe_z,
	     const double p_spindle,
	     const double p_feed,
	     const double p_plunge_feed,
	     const tool& p_t,
	     const std::vector<polyline>& p_lines);

    toolpath(const pocket_name& p_pocket_type,
	     const double p_safe_z,
	     const double p_spindle,
	     const double p_feed,
	     const double p_plunge_feed,
	     const tool& p_t,
	     const std::vector<std::vector<cut*>>& p_cuts);
    
    pocket_name pocket_type() const { return pocket_tp; }

    int tool_number() const { return t.tool_number(); }

    point start_location() const {
      DBG_ASSERT(this->lines().size() > 0);

      point start_pt = cuts.front().front()->get_start();
      return point(start_pt.x, start_pt.y, safe_z_before_tlc);
    }

    tool get_tool() const { return t; }

    point end_location() const {
      DBG_ASSERT(this->lines().size() > 0);

      point end_pt = cuts.back().back()->get_end();
      return point(end_pt.x, end_pt.y, safe_z_before_tlc);
    }

    std::vector<polyline> lines() const;

    std::vector<cut*> contiguous_cuts(const cut_params& params) const;

    std::vector<std::vector<cut*>> cuts_without_safe_moves() const {
      return cuts;
    }

  };

  double execution_time_seconds(const toolpath& tp, const double rapid_feed);
  double air_time_seconds(const toolpath& tp,
			  const double rapid_feed);


}

#endif
