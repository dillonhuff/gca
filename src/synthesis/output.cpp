#include <cmath>

#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/output.h"
#include "synthesis/safe_move.h"
#include "synthesis/shape_layout.h"

namespace gca {

  move_instr* circular_arc_to_gcode(circular_arc ca) {
    move_instr* circle_move_instr;
    if (ca.dir == CLOCKWISE) {
      circle_move_instr = g2_instr::make(lit::make(ca.end.x), lit::make(ca.end.y), lit::make(ca.end.z),
					 lit::make(ca.start_offset.x), lit::make(ca.start_offset.y), omitted::make(),
					 ca.feedrate);
    } else if (ca.dir == COUNTERCLOCKWISE) {
      circle_move_instr = g3_instr::make(lit::make(ca.end.x), lit::make(ca.end.y), lit::make(ca.end.z),
					 lit::make(ca.start_offset.x), lit::make(ca.start_offset.y), omitted::make(),
					 ca.feedrate);

    } else {
      assert(false);
    }
    return circle_move_instr;
  }

  
  linear_cut* line_to_cut(line& l, double cutter_width) {
    double w = cutter_width / 2;
    point lv = l.end - l.start;
    point v = w*(lv.rotate_z(90).normalize());
    point sp = l.start + v;
    point ep = l.end + v;
    point sr = extend_back(sp, ep, w);
    point er = extend_back(ep, sp, w);
    return linear_cut::make(sr, er);
  }

  linear_cut* vertical_cut_to(linear_cut* ct) {
    point start(ct->start.x, ct->start.y, 0);
    point end(ct->start);
    return linear_cut::make(start, end);
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
    instr* pull_up_instr = g0_instr::make(point(from.x, from.y, 0.0));
    instr* move_instr = g0_instr::make(point(to.x, to.y, 0.0));
    instr* push_down_instr = g0_instr::make(to);
    p->push_back(pull_up_instr);
    p->push_back(move_instr);
    p->push_back(push_down_instr);
  }

  void from_to_with_G1(gprog* p, point from, point to) {
    instr* move_instr = g1_instr::make(to.x, to.y, to.z);
    p->push_back(move_instr);
  }
  
  gprog* gcode_for_cuts(vector<linear_cut*>& cuts) {
    point current_loc = point(0, 0, 0);
    gprog* p = gprog::make();
    for (unsigned i = 0; i < cuts.size(); i++) {
      from_to_with_G0(p, current_loc, cuts[i]->start);
      from_to_with_G1(p, cuts[i]->start, cuts[i]->end);
      current_loc = cuts[i]->end;
    }
    point final_loc = point(0, 0, 0);
    from_to_with_G0(p, current_loc, final_loc);
    p->push_back(m2_instr::make());
    return p;
  }

  linear_cut* sink_cut(linear_cut* s, double l) {
    double xd = s->end.x - s->start.x;
    double yd = s->end.y - s->start.y;
    double x_pos = xd > 0;
    double y_pos = yd > 0;
    if (xd == 0) {
      if (y_pos) {
	return linear_cut::make(point(s->start.x, s->start.y - l, 0), s->start);
      } else {
	return linear_cut::make(point(s->start.x, s->start.y + l, 0), s->start);
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
    return linear_cut::make(point(xs, ys, 0), s->start);
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
      cuts.push_back(linear_cut::make(c_left, c_right));
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

  gprog* append_footer(gprog* p, machine_name m) {
    if (m == CAMASTER) {
      p->push_back(g53_instr::make(omitted::make(), omitted::make(), lit::make(0.0)));
      p->push_back(m5_instr::make());
    } else if (m == PROBOTIX_V90_MK2_VFD) {
      p->push_back(m2_instr::make());
    } else {
      assert(false);
    }
    return p;
  }

  void append_footer_blocks(vector<block>& blocks, machine_name m) {
    block b;
    if (m == CAMASTER) {
      b.push_back(token('G', 53));
      b.push_back(token('Z', 0.0));
      b.push_back(token('M', 5));
    } else if (m == PROBOTIX_V90_MK2_VFD) {
      b.push_back(token('M', 2));
    } else {
      assert(false);
    }
    blocks.push_back(b);
  }
  
  gprog* initial_gprog(machine_name m) {
    gprog* r = gprog::make();
    if (m == CAMASTER) {
      r->push_back(g90_instr::make());
      r->push_back(m5_instr::make());
      r->push_back(g53_instr::make(omitted::make(), omitted::make(), lit::make(0.0)));
      r->push_back(t_instr::make(6));
      r->push_back(s_instr::make(0));
      r->push_back(m3_instr::make());
      r->push_back(g53_instr::make(omitted::make(), omitted::make(), lit::make(0.0)));
      r->push_back(f_instr::make(5, "XY"));
      r->push_back(f_instr::make(5, "Z"));
    } else if (m == PROBOTIX_V90_MK2_VFD) {
      r->push_back(g90_instr::make());
    } else {
      assert(false);
    }
    return r;
  }

  void append_drill_header(gprog* p, machine_name m) {
    if (m == CAMASTER) {
      p->push_back(g90_instr::make());
      p->push_back(m5_instr::make());
      p->push_back(g53_instr::make(omitted::make(), omitted::make(), lit::make(0.0)));
      p->push_back(t_instr::make(2));
      p->push_back(s_instr::make(16000));
      p->push_back(m3_instr::make());
      p->push_back(g53_instr::make(omitted::make(), omitted::make(), lit::make(0.0)));
      p->push_back(f_instr::make(4, "XY"));
      p->push_back(f_instr::make(50, "Z"));
    } else if (m == PROBOTIX_V90_MK2_VFD) {
      p->push_back(g90_instr::make());
    } else {
      assert(false);
    }
  }

  void append_drill_header_block(vector<block>& blocks, machine_name m) {
    block b1, b2;
    // TODO: Deal with feedrates on CAMASTER
    if (m == CAMASTER) {
      b1.push_back(token('G', 90));
      b1.push_back(token('M', 5));
      b1.push_back(token('G', 53));
      b1.push_back(token('Z', 0.0));
      b2.push_back(token('G', 90));
      b2.push_back(token('S', 16000));
      b2.push_back(token('T', 2));
      b2.push_back(token('M', 3));
      // p->push_back(f_instr::make(4, "XY"));
      // p->push_back(f_instr::make(50, "Z"));
    } else if (m == PROBOTIX_V90_MK2_VFD) {
      b1.push_back(token('G', 90));
    } else {
      assert(false);
    }
    blocks.push_back(b1);
    blocks.push_back(b2);
  }

  void append_drag_knife_transfer_block(vector<block>& blocks, machine_name m) {
    block b;
    // TODO: Deal with feedrate printouts for CAMASTER
    if (m == CAMASTER) {
      b.push_back(token('M', 5));
      b.push_back(token('G', 53));
      b.push_back(token('Z', 0.0));
      b.push_back(token('G', 90));
      b.push_back(token('S', 0));
      b.push_back(token('T', 6));
      // p->push_back(f_instr::make(5, "XY"));
      // p->push_back(f_instr::make(5, "Z"));
    } else if (m == PROBOTIX_V90_MK2_VFD) {
      b.push_back(token('G', 90));
      b.push_back(token('M', 5));
      b.push_back(token('S', 0));
    } else {
      assert(false);
    }
    blocks.push_back(b);
  }
  
  void append_drag_knife_transfer(gprog* p, machine_name m) {
    if (m == CAMASTER) {
      p->push_back(g90_instr::make());
      p->push_back(g53_instr::make(omitted::make(), omitted::make(), lit::make(0.0)));
      p->push_back(m5_instr::make());
      p->push_back(t_instr::make(6));
      p->push_back(s_instr::make(0));
      p->push_back(f_instr::make(5, "XY"));
      p->push_back(f_instr::make(5, "Z"));
    } else if (m == PROBOTIX_V90_MK2_VFD) {
      p->push_back(g90_instr::make());
      p->push_back(m5_instr::make());
      p->push_back(s_instr::make(0));
    } else {
      assert(false);
    }
  }

  vector<cut*> from_to_with_G0_height(point current_loc,
				      point next_loc,
				      double safe_height,
				      value* feedrate) {
    point safe_up = current_loc;
    safe_up.z = safe_height;
    point safe_next = next_loc;
    safe_next.z = safe_height;
    vector<cut*> cuts;
    cuts.push_back(safe_move::make(current_loc, safe_up));
    cuts.push_back(safe_move::make(safe_up, safe_next));
    cuts.push_back(linear_cut::make(safe_next, next_loc));
    return cuts;
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

  tool_name get_tool_no(const cut* t) { return t->tool_no; }

  void append_cut_with_settings(const cut* last,
				const cut* next,
				gprog& p,
				const cut_params& params) {
    append_settings(last, next, p, params);
    append_cut(next, p);
  }
  
  void append_cuts_gcode(const vector<cut*>& cuts,
			 gprog& p,
			 const cut_params& params) {
    vector<tool_name> active_tools(cuts.size());
    transform(cuts.begin(), cuts.end(), active_tools.begin(), get_tool_no);
    
    cut* next_cut = NULL;
    cut* last_cut = NULL;
    for (unsigned i = 0; i < cuts.size(); i++) {
      next_cut = cuts[i];
      append_cut_with_settings(last_cut, next_cut, p, params);
      last_cut = next_cut;
    }
  }

  gprog* gcode_for_cuts(const vector<cut*>& cuts, const cut_params& params) {
    gprog* p = gprog::make();
    append_cuts_gcode(cuts, *p, params);
    gprog* r = append_footer(p, params.target_machine);
    return r;
  }

}
