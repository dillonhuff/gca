#include <cmath>

#include "gcode/circular_arc.h"
#include "gcode/safe_move.h"
#include "backend/cut_to_gcode.h"
#include "backend/output.h"
#include "utils/algorithm.h"

namespace gca {

  void append_return_home(vector<block>& bs, const cut_params& params) {
    block b;
    b.push_back(token('G', 90));
    b.push_back(token('G', 0));
    b.push_back(token('X', params.start_loc.x));
    b.push_back(token('Y', params.start_loc.y));
    b.push_back(token('Z', params.safe_height));
    bs.push_back(b);
  }

  block circular_arc_to_gcode_block(circular_arc ca) {
    block b;
    if (ca.dir == CLOCKWISE) {
      b.push_back(token('G', 2));
    } else if (ca.dir == COUNTERCLOCKWISE) {
      b.push_back(token('G', 3));
    } else {
      assert(false);
    }
    b.push_back(token('X', ca.get_end().x));
    b.push_back(token('Y', ca.get_end().y));
    b.push_back(token('Z', ca.get_end().z));
    b.push_back(token('I', ca.start_offset.x));
    b.push_back(token('J', ca.start_offset.y));
    return b;
  }
  
  void append_cut_block(const cut* ci, vector<block>& blocks) {
    block b;
    if (ci->is_hole_punch()) {
    } else if (ci->is_linear_cut()) {
      b.push_back(token('G', 1));
      b.push_back(token('X', ci->get_end().x));
      b.push_back(token('Y', ci->get_end().y));
      b.push_back(token('Z', ci->get_end().z));
      if (!ci->get_feedrate()->is_omitted()) { b.push_back(token('F', ci->get_feedrate())); }
    } else if (ci->is_circular_arc()) {
      const circular_arc* arc = static_cast<const circular_arc*>(ci);
      b = circular_arc_to_gcode_block(*arc);
    } else if (ci->is_safe_move()) {
      b.push_back(token('G', 0));
      b.push_back(token('X', ci->get_end().x));
      b.push_back(token('Y', ci->get_end().y));
      b.push_back(token('Z', ci->get_end().z));
    } else {
      assert(false);
    }
    blocks.push_back(b);
  }


  std::vector<block>
  camaster_tool_change_block(const int tool_no) {
    vector<block> blks;

    block stop_spindle_block{token('M', 5)};
    blks.push_back(stop_spindle_block);

    block lift_up_block{token('G', 53), token('Z', 0.0)};
    blks.push_back(lift_up_block);

    block tool_change_block{token('T', tool_no)};
    blks.push_back(tool_change_block);

    return blks;
    
    
  }
  std::vector<block>
  camaster_spindle_speed_blocks(const double spindle_speed) {
    vector<block> blks;

    block stop_spindle_block{token('M', 5)};
    blks.push_back(stop_spindle_block);

    block set_spindle_speed_block{token('S', spindle_speed)};
    blks.push_back(set_spindle_speed_block);

    block start_spindle_block{token('M', 3)};
    blks.push_back(start_spindle_block);

    return blks;
  }

  std::vector<block>
  wells_spindle_speed_blocks(const double spindle_speed) {
    vector<block> blks;

    block stop_spindle_block{token('M', 5)};
    blks.push_back(stop_spindle_block);

    block set_spindle_speed_block{token('S', spindle_speed)};
    blks.push_back(set_spindle_speed_block);

    block start_spindle_block{token('M', 3)};
    blks.push_back(start_spindle_block);

    return blks;
  }
  
  // TODO: This needs to be changed to do FULL machine
  // settings changes
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
	block b;
	double next_ss = get_spindle_speed(next);
	if (last == NULL || !within_eps(get_spindle_speed(last), next_ss)) {
	  if (params.target_machine == EMCO_F1) {
	    block b;
	    b.push_back(token('S', next_ss));
	    blocks.push_back(b);
	  } else if (params.target_machine == CAMASTER) {
	    concat(blocks, camaster_spindle_speed_blocks(next_ss));
	  } else if (params.target_machine == WELLS) {
	    concat(blocks, wells_spindle_speed_blocks(next_ss));
	  } else {
	    DBG_ASSERT(false);
	  }
	}
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

  void append_header_blocks(vector<block>& bs, const machine_name m) {
    if (m == EMCO_F1) {
      block b;
      b.push_back(token('G', 90));
      bs.push_back(b);
    }
  }
  
  vector<block> gcode_blocks_for_cuts(const vector<cut*>& cuts,
				      const cut_params& params) {
    vector<block> bs;
    append_header_blocks(bs, params.target_machine);
    append_cuts_gcode_blocks(cuts, bs, params);
    append_footer_blocks(bs, params.target_machine);    
    return bs;
  }
  
}
