#include <cassert>
#include <cmath>

#include "core/context.h"
#include "synthesis/align_blade.h"

namespace gca {

  double angle_between(point u, point v) {
    double rads = acos((u.dot(v)) / (u.len() * v.len()));
    return (180.0/M_PI)*rads;
  }

  void align_coords(point desired_orient,
		    point circle_end,
		    point current_orient,
		    double rad,
		    point& circle_start,
		    point& circle_start_off) {
    double theta = angle_between(desired_orient, current_orient);
    cout << "angle between desired and current orient = " << theta << endl;
    point ef = rad*desired_orient.normalize();
    point circle_center = circle_end - ef;
    cout << "Computed circle center: " << circle_center << endl;
    circle_start_off = -1*ef.rotate_z(theta);
    cout << "circle start offset " << circle_start_off << endl;
    circle_start = circle_center - circle_start_off;
    if (!within_eps(circle_start_off.len(), rad, 0.00001)) {
      cout << "incorrect center offset: " << circle_start_off << endl;
      cout << "bad center offset length: " << circle_start_off.len() << endl;
      cout << "rad = " << rad << endl;
      assert(false);
    }
  }

  void from_to_with_G0_drag_knife(double safe_height,
				  double align_depth,
				  gprog* p,
				  point last_pos,
				  point last_orient,
				  point next_pos,
				  point next_orient) {
    instr* pull_up_instr = mk_G0(point(last_pos.x, last_pos.y, safe_height));
    // TODO: Set this to realistic value
    double r = 0.16;
    point c_pos;
    point circle_center_offset;
    point next_pos_xy = next_pos;
    next_pos_xy.z = align_depth;
    align_coords(next_orient, next_pos_xy, last_orient, r, c_pos, circle_center_offset);
    instr* move_to_c_pos_instr = mk_G0(c_pos.x, c_pos.y, safe_height);
    instr* push_down_instr = mk_G1(c_pos.x, c_pos.y, align_depth, mk_omitted());
    // Computing signed angle
    point center = c_pos + circle_center_offset;
    point sv = -1 * circle_center_offset;
    point ev = next_pos - center;
    double angle = atan2(ev.y, ev.x) - atan2(sv.y, sv.x);
    instr* circle_move_instr;
    cout << "Angle between = " << angle << endl;
    if (angle < 0) {
      circle_move_instr = mk_G2(mk_lit(next_pos.x), mk_lit(next_pos.y), mk_omitted(),
				mk_lit(circle_center_offset.x), mk_lit(circle_center_offset.y), mk_omitted(),
				mk_omitted());
    } else {
      circle_move_instr = mk_G3(mk_lit(next_pos.x), mk_lit(next_pos.y), mk_omitted(),
				mk_lit(circle_center_offset.x), mk_lit(circle_center_offset.y), mk_omitted(),
				mk_omitted());

    }
    instr* final_push_down_instr = mk_G1(next_pos.x, next_pos.y, next_pos.z, mk_omitted());
    p->push_back(pull_up_instr);
    p->push_back(move_to_c_pos_instr);
    p->push_back(push_down_instr);
    p->push_back(circle_move_instr);
    p->push_back(final_push_down_instr);
  }
  
}
