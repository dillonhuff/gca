#include <cassert>
#include <cmath>


#include "backend/align_blade.h"
#include "gcode/linear_cut.h"
#include "gcode/safe_move.h"

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
    circular_arc ca = circular_arc(circle_start, circle_end, circle_start_off, CLOCKWISE, XY);
    point sv = ca.center_to_start_vec();
    point ev = ca.center_to_end_vec();
    double angle = atan2(ev.y, ev.x) - atan2(sv.y, sv.x);
    if (angle < 0) {
      ca.dir = CLOCKWISE;
    } else {
      ca.dir = COUNTERCLOCKWISE;
    }
    return ca;
  }

  vector<cut*> from_to_with_G0_drag_knife(double safe_height,
					  double align_depth,
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
    point sd = point(ca.get_start().x, ca.get_start().y, safe_height);
    point rd = point(ca.get_start().x, ca.get_start().y, align_depth);
    cuts.push_back(safe_move::make(last_pos_up, sd));
    cuts.push_back(linear_cut::make(sd, rd));
    cuts.push_back(circular_arc::make(ca.get_start(), ca.get_end(), ca.start_offset, ca.dir, ca.pl));
    cuts.push_back(linear_cut::make(ca.get_end(), next_pos));
    return cuts;
  }
  
}
