#include <cmath>

#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/output.h"
#include "synthesis/safe_move.h"
#include "synthesis/shape_layout.h"

namespace gca {

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
    auto c = linear_cut::make(safe_next, next_loc);
    c->set_feedrate(feedrate);
    cuts.push_back(c);
    return cuts;
  }

}
