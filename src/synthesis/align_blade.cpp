#include <cassert>
#include <cmath>

#include "core/context.h"
#include "synthesis/align_blade.h"

namespace gca {

  double angle_between(point u, point v) {
    double rads = acos((u.dot(v)) / (u.len() * v.len()));
    return (180.0/M_PI)*rads;
  }

  circular_arc align_coords(point desired_orient,
			    point circle_end,
			    point current_orient,
			    double rad) {
    double theta = angle_between(desired_orient, current_orient);
    cout << "angle between desired and current orient = " << theta << endl;
    point ef = rad*desired_orient.normalize();
    point circle_center = circle_end - ef;
    cout << "Computed circle center: " << circle_center << endl;
    point circle_start_off = -1*ef.rotate_z(theta);
    cout << "circle start offset " << circle_start_off << endl;
    point circle_start = circle_center - circle_start_off;
    if (!within_eps(circle_start_off.len(), rad, 0.00001)) {
      cout << "incorrect center offset: " << circle_start_off << endl;
      cout << "bad center offset length: " << circle_start_off.len() << endl;
      cout << "rad = " << rad << endl;
      assert(false);
    }
    return circular_arc(circle_start, circle_end, circle_start_off);
  }

  move_instr* circular_arc_to_gcode(circular_arc ca) {
    // Computing signed angle
    point sv = -1 * ca.start_offset;
    point ev = ca.end - ca.center();
    double angle = atan2(ev.y, ev.x) - atan2(sv.y, sv.x);
    cout << "Angle between = " << angle << endl;
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
