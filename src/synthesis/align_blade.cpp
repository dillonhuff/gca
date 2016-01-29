#include <cassert>
#include <cmath>

#include "core/context.h"
#include "synthesis/align_blade.h"

namespace gca {

  double angle_between(point u, point v) {
    double rads = acos((u.dot(v)) / (u.len() * v.len()));
    return (180.0/M_PI)*rads;
  }

  void align_coords(point desired_dir,
		    point desired_pos,
		    point current_dir,
		    double rad,
		    point& c_pos,
		    point& center_off) {
    assert(desired_dir.z == 0);
    assert(current_dir.z == 0);
    assert(desired_pos.z == 0);
    double theta = -1.0 * angle_between(current_dir, desired_dir);
    point s = desired_pos - (rad * desired_pos.normalize());
    center_off = point(0, 1, 0);
    point c_pos_prime = (desired_pos - s).rotate_z(theta);
    c_pos = s + c_pos_prime;
    center_off = -1.0 * (c_pos - s);
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
    double r = 0.5;
    point c_pos;
    point circle_center_offset;
    point next_pos_xy = next_pos;
    next_pos_xy.z = 0;
    align_coords(next_orient, next_pos_xy, last_orient, r, c_pos, circle_center_offset);
    instr* move_to_c_pos_instr = mk_G0(c_pos.x, c_pos.y, safe_height);
    instr* push_down_instr = mk_G1(c_pos.x, c_pos.y, align_depth, mk_omitted());
    instr* circle_move_instr = mk_G3(mk_lit(next_pos.x), mk_lit(next_pos.y), mk_omitted(),
				     mk_lit(circle_center_offset.x), mk_lit(circle_center_offset.y), mk_omitted(),
				     mk_omitted());
    instr* final_push_down_instr = mk_G1(next_pos.x, next_pos.y, next_pos.z, mk_omitted());
    p->push_back(pull_up_instr);
    p->push_back(move_to_c_pos_instr);
    p->push_back(push_down_instr);
    p->push_back(circle_move_instr);
    p->push_back(final_push_down_instr);
  }
  
}
