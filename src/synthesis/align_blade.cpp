#include <cassert>
#include <cmath>

#include "core/context.h"
#include "synthesis/align_blade.h"
#include "synthesis/safe_move.h"

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
    return circular_arc(circle_start, circle_end, circle_start_off, CLOCKWISE, XY);
  }

  vector<cut*> from_to_with_G0_drag_knife(double safe_height,
					  double align_depth,
					  gprog* p,
					  point last_pos,
					  point last_orient,
					  point next_pos,
					  point next_orient) {
    vector<cut*> cuts;
    point last_pos_up = last_pos;
    last_pos_up.z = safe_height;
    cuts.push_back(safe_move::make(last_pos, last_pos_up));
    double r = 0.16;
    point next_pos_xy = next_pos;
    next_pos_xy.z = align_depth;
    circular_arc ca = align_coords(next_orient, next_pos_xy, last_orient, r);
    point sd = point(ca.start.x, ca.start.y, safe_height);
    point rd = point(ca.start.x, ca.start.y, align_depth);
    cuts.push_back(safe_move::make(last_pos_up, sd));
    cuts.push_back(linear_cut::make(sd, rd));
    cuts.push_back(circular_arc::make(ca.start, ca.end, ca.start_offset, ca.dir, ca.pl));
    cuts.push_back(linear_cut::make(ca.end, next_pos));
    return cuts;
    // instr* pull_up_instr = mk_G0(point(last_pos.x, last_pos.y, safe_height));
    // double r = 0.16;
    // point next_pos_xy = next_pos;
    // next_pos_xy.z = align_depth;
    // circular_arc ca = align_coords(next_orient, next_pos_xy, last_orient, r);
    // instr* move_to_c_pos_instr = mk_G0(ca.start.x, ca.start.y, safe_height);
    // instr* push_down_instr = mk_G1(ca.start.x, ca.start.y, align_depth, mk_omitted());
    // instr* circle_move_instr = circular_arc_to_gcode(ca);
    // instr* final_push_down_instr = mk_G1(next_pos.x, next_pos.y, next_pos.z, mk_omitted());
    // p->push_back(pull_up_instr);
    // p->push_back(move_to_c_pos_instr);
    // p->push_back(push_down_instr);
    // p->push_back(circle_move_instr);
    // p->push_back(final_push_down_instr);
  }
  
}
