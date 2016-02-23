#include <cmath>

#include "output.h"

namespace gca {

  linear_cut* line_to_cut(line& l, double cutter_width) {
    double w = cutter_width / 2;
    point lv = l.end - l.start;
    point v = w*(lv.rotate_z(90).normalize());
    point sp = l.start + v;
    point ep = l.end + v;
    point sr = extend_back(sp, ep, w);
    point er = extend_back(ep, sp, w);
    return mk_linear_cut(sr, er);
  }

  linear_cut* vertical_cut_to(linear_cut* ct) {
    point start(ct->start.x, ct->start.y, 0);
    point end(ct->start);
    return mk_linear_cut(start, end);
  }

  vector<linear_cut*> lines_to_cuts(vector<line>& lines, double cutter_width) {
    vector<linear_cut*> cuts;
    for (unsigned i = 0; i < lines.size(); i++) {
      linear_cut* res = line_to_cut(lines[i], cutter_width);
      linear_cut* down = vertical_cut_to(res);
      cuts.push_back(down);
      cuts.push_back(res);
    }
    return cuts;
  }
  
  void from_to_with_G0(gprog* p, point from, point to) {
    instr* pull_up_instr = mk_G0(point(from.x, from.y, 0.0));
    instr* move_instr = mk_G0(point(to.x, to.y, 0.0));
    instr* push_down_instr = mk_G0(to);
    p->push_back(pull_up_instr);
    p->push_back(move_instr);
    p->push_back(push_down_instr);
  }

  void from_to_with_G1(gprog* p, point from, point to) {
    instr* move_instr = mk_G1(to.x, to.y, to.z);
    p->push_back(move_instr);
  }
  
  gprog* gcode_for_cuts(vector<linear_cut*>& cuts) {
    point current_loc = point(0, 0, 0);
    gprog* p = mk_gprog();
    for (unsigned i = 0; i < cuts.size(); i++) {
      from_to_with_G0(p, current_loc, cuts[i]->start);
      from_to_with_G1(p, cuts[i]->start, cuts[i]->end);
      current_loc = cuts[i]->end;
    }
    point final_loc = point(0, 0, 0);
    from_to_with_G0(p, current_loc, final_loc);
    p->push_back(mk_m2_instr());
    return p;
  }

  linear_cut* sink_cut(linear_cut* s, double l) {
    double xd = s->end.x - s->start.x;
    double yd = s->end.y - s->start.y;
    double x_pos = xd > 0;
    double y_pos = yd > 0;
    if (xd == 0) {
      if (y_pos) {
	return mk_linear_cut(point(s->start.x, s->start.y - l, 0), s->start);
      } else {
	return mk_linear_cut(point(s->start.x, s->start.y + l, 0), s->start);
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
    return mk_linear_cut(point(xs, ys, 0), s->start);
  }

  void insert_sink_cuts(double l, vector<linear_cut*>& cuts, vector<linear_cut*>& dest) {
    for (unsigned i = 0; i < cuts.size(); i++) {
      dest.push_back(sink_cut(cuts[i], l));
      dest.push_back(cuts[i]);
    }
  }

  vector<linear_cut*> surface_cuts(point left, point right,
				   point shift, int num_cuts) {
    vector<linear_cut*> cuts;
    point c_left = left;
    point c_right = right;
    for (int i = 1; i <= num_cuts; i++) {
      cuts.push_back(mk_linear_cut(c_left, c_right));
      // Reverse cut directions
      point temp = c_left;
      c_left = c_right + shift;
      c_right = temp + shift;
    }
    return cuts;
  }


  vector<linear_cut*> two_pass_surface(double coarse_depth, double finish_inc,
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
    
    vector<linear_cut*> cuts1 = surface_cuts(coarse_start, coarse_end,
					     shift, num_cuts);
    vector<linear_cut*> cuts2 = surface_cuts(finish_start, finish_end,
					     shift, num_cuts);
    cuts1.insert(cuts1.end(), cuts2.begin(), cuts2.end());
    return cuts1;
  }

  gprog* append_footer(gprog* p) {
    //p->push_back(mk_G53(mk_omitted(), mk_omitted(), mk_lit(0.0)));
    p->push_back(mk_m2_instr());
    return p;
  }

  gprog* initial_gprog() {
    gprog* r = mk_gprog();
    r->push_back(mk_G90());
    //r->push_back(mk_m5_instr());
    //r->push_back(mk_G53(mk_omitted(), mk_omitted(), mk_lit(0.0)));
    //r->push_back(mk_t_instr(6));
    //r->push_back(mk_s_instr(0));
    //r->push_back(mk_m3_instr());
    //r->push_back(mk_G53(mk_omitted(), mk_omitted(), mk_lit(0.0)));
    //r->push_back(mk_f_instr(5, "XY"));
    //r->push_back(mk_f_instr(5, "Z"));
    return r;
  }

  void append_drill_header(gprog* p) {
    p->push_back(mk_G90());
    //p->push_back(mk_m5_instr());
    //p->push_back(mk_G53(mk_omitted(), mk_omitted(), mk_lit(0.0)));
    //p->push_back(mk_t_instr(2));
    //p->push_back(mk_s_instr(16000));
    //p->push_back(mk_m3_instr());
    //p->push_back(mk_G53(mk_omitted(), mk_omitted(), mk_lit(0.0)));
    //p->push_back(mk_f_instr(4, "XY"));
    //p->push_back(mk_f_instr(50, "Z"));
  }

  void append_drag_knife_transfer(gprog* p) {
    p->push_back(mk_G53(mk_omitted(), mk_omitted(), mk_lit(0.0)));
    p->push_back(mk_m5_instr());
    p->push_back(mk_t_instr(6));
    p->push_back(mk_s_instr(0));
    p->push_back(mk_f_instr(5, "XY"));
    p->push_back(mk_f_instr(5, "Z"));
  }

  void from_to_with_G0_height(gprog* p,
			      point current_loc,
			      point next_loc,
			      double safe_height,
			      value* feedrate) {
    g0_instr* pull_up_instr = mk_G0(current_loc.x, current_loc.y, safe_height);
    g0_instr* move_xy_instr = mk_G0(next_loc.x, next_loc.y, safe_height);
    g1_instr* push_down_instr = mk_G1(next_loc.x, next_loc.y, next_loc.z, feedrate);
    p->push_back(pull_up_instr);
    p->push_back(move_xy_instr);
    p->push_back(push_down_instr);
  }

}
