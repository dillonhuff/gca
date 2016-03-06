#include <cmath>


#include "synthesis/cut_to_gcode.h"
#include "synthesis/circular_arc.h"
#include "synthesis/output.h"
#include "synthesis/safe_move.h"

namespace gca {

  move_instr* circular_arc_to_gcode(circular_arc ca) {
    point sv = ca.center_to_start_vec();
    point ev = ca.center_to_end_vec();
    double angle = atan2(ev.y, ev.x) - atan2(sv.y, sv.x);
    move_instr* circle_move_instr;
    if (angle < 0) {
      circle_move_instr = g2_instr::make(lit::make(ca.end.x), lit::make(ca.end.y), omitted::make(),
					 lit::make(ca.start_offset.x), lit::make(ca.start_offset.y), omitted::make(),
					 omitted::make());
    } else {
      circle_move_instr = g3_instr::make(lit::make(ca.end.x), lit::make(ca.end.y), omitted::make(),
					 lit::make(ca.start_offset.x), lit::make(ca.start_offset.y), omitted::make(),
					 omitted::make());

    }
    return circle_move_instr;
  }

  void append_cut(const cut* ci, gprog& p) {
    if (ci->is_hole_punch()) {
    } else if (ci->is_linear_cut()) {
      p.push_back(g1_instr::make(ci->end.x, ci->end.y, ci->end.z, ci->feedrate));
    } else if (ci->is_circular_arc()) {
      const circular_arc* arc = static_cast<const circular_arc*>(ci);
      p.push_back(circular_arc_to_gcode(*arc));
    } else if (ci->is_safe_move()) {
      p.push_back(g0_instr::make(ci->end));
    } else {
      assert(false);
    }    
  }

  void append_settings(const cut* last,
		       const cut* next,
		       gprog& p,
		       const cut_params& params) {
    if (last == NULL || last->tool_no != next->tool_no) {
      if (next->tool_no == DRAG_KNIFE) {
	append_drag_knife_transfer(&p, params.target_machine);
      } else if (next->tool_no == DRILL) {
	append_drill_header(&p, params.target_machine);
      } else {
	assert(false);
      }
    }
  }

  void append_cut_with_settings(const cut* last,
				const cut* next,
				gprog& p,
				const cut_params& params) {
    append_settings(last, next, p, params);
    append_cut(next, p);
  }
  
}
