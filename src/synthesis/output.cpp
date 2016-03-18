#include <cmath>

#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/output.h"
#include "synthesis/safe_move.h"
#include "synthesis/shape_layout.h"

namespace gca {

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

}
