#include <cmath>

#include "core/context.h"
#include "synthesis/cut_to_gcode.h"
#include "synthesis/circular_arc.h"
#include "synthesis/safe_move.h"

namespace gca {

  move_instr* circular_arc_to_gcode(circular_arc ca) {
    point sv = ca.center_to_start_vec();
    point ev = ca.center_to_end_vec();
    double angle = atan2(ev.y, ev.x) - atan2(sv.y, sv.x);
    move_instr* circle_move_instr;
    if (angle < 0) {
      circle_move_instr = mk_G2(lit::make(ca.end.x), lit::make(ca.end.y), omitted::make(),
				lit::make(ca.start_offset.x), lit::make(ca.start_offset.y), omitted::make(),
				omitted::make());
    } else {
      circle_move_instr = mk_G3(lit::make(ca.end.x), lit::make(ca.end.y), omitted::make(),
				lit::make(ca.start_offset.x), lit::make(ca.start_offset.y), omitted::make(),
				omitted::make());

    }
    return circle_move_instr;
  }

  void append_cut(cut* ci, gprog& p) {
    if (ci->is_hole_punch()) {
    } else if (ci->is_linear_cut()) {
      p.push_back(mk_G1(ci->end.x, ci->end.y, ci->end.z, ci->feedrate));
    } else if (ci->is_circular_arc()) {
      circular_arc* arc = static_cast<circular_arc*>(ci);
      p.push_back(circular_arc_to_gcode(*arc));
    } else if (ci->is_safe_move()) {
      p.push_back(mk_G0(ci->end));
    } else {
      assert(false);
    }    
  }
  
}
