#include <cassert>
#include <cmath>

#include "core/context.h"
#include "synthesis/align_blade.h"

namespace gca {

  circular_arc align_coords(point desired_orient,
			    point circle_end,
			    point current_orient,
			    double rad) {
    double theta = angle_between(desired_orient, current_orient);
    point ef = rad*desired_orient.normalize();
    point circle_center = circle_end - ef;
    point circle_start_off = -1*ef.rotate_z(theta);
    point circle_start = circle_center - circle_start_off;
    if (!within_eps(circle_start_off.len(), rad, 0.00001)) {
      cout << "incorrect center offset: " << circle_start_off << endl;
      cout << "bad center offset length: " << circle_start_off.len() << endl;
      cout << "rad = " << rad << endl;
      assert(false);
    }
    // TODO: Set arc direction here
    return circular_arc(circle_start, circle_end, circle_start_off, CLOCKWISE);
  }

  move_instr* circular_arc_to_gcode(circular_arc ca) {
    point sv = ca.center_to_start_vec();
    point ev = ca.center_to_end_vec();
    double angle = atan2(ev.y, ev.x) - atan2(sv.y, sv.x);
    move_instr* circle_move_instr;
    if (angle < 0) {
      circle_move_instr = mk_G2(mk_lit(ca.end.x), mk_lit(ca.end.y), mk_omitted(),
				mk_lit(ca.start_offset.x), mk_lit(ca.start_offset.y), mk_omitted(),
				mk_omitted());
    } else {
      circle_move_instr = mk_G3(mk_lit(ca.end.x), mk_lit(ca.end.y), mk_omitted(),
				mk_lit(ca.start_offset.x), mk_lit(ca.start_offset.y), mk_omitted(),
				mk_omitted());

    }
    return circle_move_instr;
  }

  void from_to_with_G0_drag_knife(double safe_height,
				  double align_depth,
				  gprog* p,
				  point last_pos,
				  point last_orient,
				  point next_pos,
				  point next_orient) {
    instr* pull_up_instr = mk_G0(point(last_pos.x, last_pos.y, safe_height));
    double r = 0.16;
    point next_pos_xy = next_pos;
    next_pos_xy.z = align_depth;
    circular_arc ca = align_coords(next_orient, next_pos_xy, last_orient, r);
    instr* move_to_c_pos_instr = mk_G0(ca.start.x, ca.start.y, safe_height);
    instr* push_down_instr = mk_G1(ca.start.x, ca.start.y, align_depth, mk_omitted());
    instr* circle_move_instr = circular_arc_to_gcode(ca);
    instr* final_push_down_instr = mk_G1(next_pos.x, next_pos.y, next_pos.z, mk_omitted());
    p->push_back(pull_up_instr);
    p->push_back(move_to_c_pos_instr);
    p->push_back(push_down_instr);
    p->push_back(circle_move_instr);
    p->push_back(final_push_down_instr);
  }
  
}
