#include <cmath>

#include "synthesis/cut_to_gcode.h"
#include "synthesis/circular_arc.h"
#include "synthesis/output.h"
#include "synthesis/safe_move.h"

namespace gca {

  block circular_arc_to_gcode_block(circular_arc ca) {
    block b;
    if (ca.dir == CLOCKWISE) {
      b.push_back(token('G', 2));
    } else if (ca.dir == COUNTERCLOCKWISE) {
      b.push_back(token('G', 3));
    } else {
      assert(false);
    }
    b.push_back(token('X', ca.end.x));
    b.push_back(token('Y', ca.end.y));
    b.push_back(token('Z', ca.end.z));
    b.push_back(token('I', ca.start_offset.x));
    b.push_back(token('J', ca.start_offset.y));
    return b;
  }
  
  void append_cut_block(const cut* ci, vector<block>& blocks) {
    block b;
    if (ci->is_hole_punch()) {
    } else if (ci->is_linear_cut()) {
      b.push_back(token('G', 1));
      b.push_back(token('X', ci->end.x));
      b.push_back(token('Y', ci->end.y));
      b.push_back(token('Z', ci->end.z));
      if (!ci->feedrate->is_omitted()) { b.push_back(token('F', ci->feedrate)); }
    } else if (ci->is_circular_arc()) {
      const circular_arc* arc = static_cast<const circular_arc*>(ci);
      b = circular_arc_to_gcode_block(*arc);
    } else if (ci->is_safe_move()) {
      b.push_back(token('G', 0));
      b.push_back(token('X', ci->end.x));
      b.push_back(token('Y', ci->end.y));
      b.push_back(token('Z', ci->end.z));
    } else {
      assert(false);
    }
    blocks.push_back(b);
  }

  void append_settings_block(const cut* last,
			     const cut* next,
			     vector<block>& blocks,
			     const cut_params& params) {
    if (last == NULL || last->tool_no != next->tool_no) {
      if (next->tool_no == DRAG_KNIFE) {
    	append_drag_knife_transfer_block(blocks, params.target_machine);
      } else if (next->tool_no == DRILL) {
    	append_drill_header_block(blocks, params.target_machine);
      } else {
    	assert(false);
      }
    }
  }

  void append_cut_blocks_with_settings(const cut* last,
				       const cut* next,
				       vector<block>& blocks,
				       const cut_params& params) {
    append_settings_block(last, next, blocks, params);
    append_cut_block(next, blocks);
  }
  
  void append_cuts_gcode_blocks(const vector<cut*>& cuts,
				vector<block>& blocks,
				const cut_params& params) {
    vector<tool_name> active_tools(cuts.size());
    transform(cuts.begin(), cuts.end(),
	      active_tools.begin(),
	      [](const cut* t) { return t->tool_no; });
    
    cut* next_cut = NULL;
    cut* last_cut = NULL;
    for (unsigned i = 0; i < cuts.size(); i++) {
      next_cut = cuts[i];
      append_cut_blocks_with_settings(last_cut, next_cut, blocks, params);
      last_cut = next_cut;
    }
  }
  
  vector<block> gcode_blocks_for_cuts(const vector<cut*>& cuts,
				      const cut_params& params) {
    vector<block> bs;
    append_cuts_gcode_blocks(cuts, bs, params);
    append_footer_blocks(bs, params.target_machine);
    return bs;
  }
  
}
