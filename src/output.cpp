#include <cmath>

#include "output.h"

namespace gca {

  void from_to_with_G0(context& c, gprog* p, point from, point to) {
    instr* pull_up_instr = c.mk_G0(point(from.x, from.y, 0.0));
    instr* move_instr = c.mk_G0(point(to.x, to.y, 0.0));
    instr* push_down_instr = c.mk_G0(to);
    p->push_back(pull_up_instr);
    p->push_back(move_instr);
    p->push_back(push_down_instr);
  }

  void from_to_with_G1(context& c, gprog* p, point from, point to) {
    instr* move_instr = c.mk_G1(to.x, to.y, to.z);
    p->push_back(move_instr);
  }
  
  gprog* gcode_for_cuts(context& c, vector<cut*>& cuts) {
    point current_loc = point(0, 0, 0);
    gprog* p = c.mk_gprog();
    for (int i = 0; i < cuts.size(); i++) {
      from_to_with_G0(c, p, current_loc, cuts[i]->start);
      from_to_with_G1(c, p, cuts[i]->start, cuts[i]->end);
      current_loc = cuts[i]->end;
    }
    point final_loc = point(0, 0, 0);
    from_to_with_G0(c, p, current_loc, final_loc);
    p->push_back(c.mk_minstr(2));
    return p;
  }

  cut* sink_cut(context& c, cut* s, double l) {
    double xd = s->end.x - s->start.x;
    double yd = s->end.y - s->start.y;
    double x_pos = xd > 0;
    double y_pos = yd > 0;
    if (xd == 0) {
      if (y_pos) {
	return c.mk_cut(point(s->start.x, s->start.y - l, 0), s->start);
      } else {
	return c.mk_cut(point(s->start.x, s->start.y + l, 0), s->start);
      }
    }
    double m = yd / xd;
    double a = sqrt((l*l) / (1.0 + m*m));
    double b = m*a;
    double xs, ys;
    if (x_pos) {
      xs = s->start.x - abs(a);
    } else {
      xs = s->start.x + abs(a);
    }
    if (y_pos) {
      ys = s->start.y - abs(b);
    } else {
      ys = s->start.y + abs(b);
    }
    return c.mk_cut(point(xs, ys, 0), s->start);
  }

  void insert_sink_cuts(context& c, double l, vector<cut*>& cuts, vector<cut*>& dest) {
    for (int i = 0; i < cuts.size(); i++) {
      dest.push_back(sink_cut(c, cuts[i], l));
      dest.push_back(cuts[i]);
    }
  }

  vector<cut*> surface_cuts(context &c,
			    point left, point right,
			    point shift, int num_cuts) {
    vector<cut*> cuts;
    point c_left = left;
    point c_right = right;
    for (int i = 1; i <= num_cuts; i++) {
      cuts.push_back(c.mk_cut(c_left, c_right));
      // Reverse cut directions
      point temp = c_left;
      c_left = c_right + shift;
      c_right = temp + shift;
    }
    return cuts;
  }


  vector<cut*> two_pass_surface(double coarse_depth, double finish_inc,
				double cutter_width,
				double x_s, double x_e, double y,
				double width) {
    double inc = cutter_width*(2.0/3.0);
    int num_cuts = ceil(width / inc);
    point coarse_start(x_s, y, coarse_depth);
    point coarse_end(x_e, y, coarse_depth);
    point shift(0, inc, 0);
    double finish_depth = coarse_depth + finish_inc;
    point finish_start(x_s, y, finish_depth);
    point finish_end(x_e, y, finish_depth);
    context c;
    vector<cut*> cuts1 = surface_cuts(c, coarse_start, coarse_end,
				      shift, num_cuts);
    vector<cut*> cuts2 = surface_cuts(c, finish_start, finish_end,
				      shift, num_cuts);
    cuts1.insert(cuts1.end(), cuts2.begin(), cuts2.end());
    return cuts1;
  }
  
}
