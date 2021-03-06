#include <algorithm>
#include <numeric>
#include <sstream>

#include "backend/cut_to_gcode.h"
#include "gcode/safe_move.h"
#include "backend/shapes_to_gcode.h"
#include "backend/shapes_to_toolpaths.h"

using namespace std;

namespace gca {

  vector<cut*> move_to_next_cut_dn(cut* last_cut,
				   cut* next_cut,
				   cut_params params) {
    point current_loc;
    point current_orient;
    if (last_cut == NULL || last_cut->tool_no != next_cut->tool_no) {
      current_loc = params.start_loc;
      current_orient = params.start_orient;
    } else {
      current_loc = last_cut->get_end();
      current_orient = last_cut->final_orient();
    }

    double align_depth = params.material_depth - params.push_depth;
    point next_orient = next_cut->initial_orient();
    point next_loc = next_cut->get_start();
    vector<cut*> tcuts;
    if (!within_eps(angle_between(current_orient, next_orient), 0, params.max_orientation_diff)) {
      tcuts = from_to_with_G0_drag_knife(params.safe_height,
					 align_depth,
					 current_loc,
					 current_orient,
					 next_loc,
					 next_orient);
    } else if (!within_eps(current_loc, next_loc)) {
      tcuts = from_to_with_G0_height(current_loc, next_cut->get_start(), params.safe_height, lit::make(params.default_feedrate));
    }
    return tcuts;
  }

  vector<cut*> move_to_next_cut_drill(cut* last_cut,
				      cut* next_cut,
				      const cut_params& params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->get_end();

    vector<cut*> tcuts;
    lit* feed;
    if (params.plunge_feed_is_set()) {
      feed = lit::make(params.plunge_feed());
    } else {
      feed = lit::make(params.default_feedrate);
    }

    if (!within_eps(current_loc, next_cut->get_start())) {
      tcuts =
	from_to_with_G0_height(current_loc,
			       next_cut->get_start(),
			       params.safe_height,
			       feed);
    }
    return tcuts;
  }

  // TODO: Rework this code to avoid the vestigial cut->tool_no parameter
  vector<cut*> move_to_next_cut(cut* last_cut,
				cut* next_cut,
				const cut_params& params) {
    vector<cut*> trans;

    if (next_cut->tool_no == DRAG_KNIFE) {
      trans = move_to_next_cut_dn(last_cut, next_cut, params);
    } else {
      trans = move_to_next_cut_drill(last_cut, next_cut, params);
    }

    lit* feed;
    if (params.plunge_feed_is_set()) {
      feed = lit::make(params.plunge_feed());
    } else {
      feed = lit::make(params.default_feedrate);
    }

    for (auto t : trans) {
      t->set_spindle_speed(next_cut->get_spindle_speed());
      t->set_feedrate(feed); //next_cut->get_feedrate());
    }
    return trans;
  }

  void insert_move_home(vector<cut*>& cuts,
			const cut_params& params) {
    if (cuts.size() > 0) {
      point above = cuts.back()->get_end();
      above.z = params.safe_height;
      point dest = params.start_loc;
      dest.z = params.safe_height;
      cut* pull_up = safe_move::make(cuts.back()->get_end(), above);
      pull_up->tool_no = cuts.back()->tool_no;
      cut* shift = safe_move::make(above, dest);
      shift->tool_no = cuts.back()->tool_no;
      cuts.push_back(pull_up);
      // TODO: Restore?
      //      cuts.push_back(shift);
    }
  }

  vector<cut*> insert_transitions(const vector<cut*>& cuts,
				  const cut_params& params) {
    vector<cut*> all_cuts;
    cut* last_cut = NULL;
    for (auto next_cut : cuts) {
      vector<cut*> transition = move_to_next_cut(last_cut, next_cut, params);
      for (auto jt : transition) {
	jt->tool_no = next_cut->tool_no;
      }
      all_cuts.insert(all_cuts.end(), transition.begin(), transition.end());
      all_cuts.push_back(next_cut);
      last_cut = next_cut;
    }
    insert_move_home(all_cuts, params);
    return all_cuts;
  }

  bool cuts_are_adjacent(const vector<cut*>& cuts) {
    if (cuts.size() > 0) {
      for (unsigned i = 0; i < cuts.size() - 1; i++) {
	cut* current = cuts[i];
	cut* next = cuts[i+1];
	if (!within_eps(current->get_end(), next->get_start()) &&
	    current->tool_no == next->tool_no) {
	  cout << "Error: " << current->get_end() << " and " << endl;
	  cout << next->get_start() << " are not adjacent" << endl;
	}
      }
    }
    return true;
  }

  vector<cut*> shift_cuts(const vector<cut*>& cuts, point p) {
    vector<cut*> shifted_cuts;
    for (auto cut : cuts)
      { shifted_cuts.push_back(cut->shift(p)); }
    return shifted_cuts;
  }

  void set_feedrates(vector<cut*>& cuts,
		     const cut_params& params) {
    if (params.set_default_feedrate) {
      for (auto cut : cuts)
    	{ cut->set_feedrate(lit::make(params.default_feedrate)); }
    }    
  }

  vector<cut*> shape_layout_to_cuts(const shape_layout& shapes_to_cut,
				    const cut_params& params) {
    vector<cut*> scuts = shape_cuts(shapes_to_cut, params);
    vector<cut*> all_cuts = insert_transitions(scuts, params);
    assert(cuts_are_adjacent(all_cuts));
    point shift(0, 0, params.machine_z_zero);
    vector<cut*> shifted_cuts = shift_cuts(all_cuts, shift);
    set_feedrates(shifted_cuts, params);
    return shifted_cuts;
  }

  vector<block> cuts_to_gcode_no_transitions(const vector<cut*>& all_cuts,
					     const cut_params& params) {
    assert(cuts_are_adjacent(all_cuts));
    point shift(0, 0, params.machine_z_zero);
    vector<cut*> shifted_cuts = shift_cuts(all_cuts, shift);
    set_feedrates(shifted_cuts, params);
    return gcode_blocks_for_cuts(shifted_cuts, params);
  }

  // TODO: Should not need the shift in the new code generation scheme
  vector<block> cuts_to_gcode(const vector<cut*>& cuts,
			      const cut_params& params) {
    vector<cut*> all_cuts = insert_transitions(cuts, params);
    assert(cuts_are_adjacent(all_cuts));
    point shift(0, 0, params.machine_z_zero);
    vector<cut*> shifted_cuts = shift_cuts(all_cuts, shift);
    set_feedrates(shifted_cuts, params);
    return gcode_blocks_for_cuts(shifted_cuts, params);
  }

  string cuts_to_gcode_string(const vector<cut*>& cuts,
			      const cut_params& params) {
    vector<cut*> all_cuts = insert_transitions(cuts, params);
    assert(cuts_are_adjacent(all_cuts));
    point shift(0, 0, params.machine_z_zero);
    vector<cut*> shifted_cuts = shift_cuts(all_cuts, shift);
    set_feedrates(shifted_cuts, params);
    vector<block> p = gcode_blocks_for_cuts(shifted_cuts, params);
    stringstream ss;
    ss.setf(ios::fixed, ios::floatfield);
    ss.setf(ios::showpoint);
    ss << p << endl;
    return ss.str();
  }

  vector<block> shape_layout_to_gcode(const shape_layout& shapes_to_cut,
				      const cut_params& params) {
    vector<cut*> shifted_cuts = shape_layout_to_cuts(shapes_to_cut, params);
    return gcode_blocks_for_cuts(shifted_cuts, params);
  }
  
  string shape_layout_to_gcode_string(const shape_layout& shapes_to_cut,
				      const cut_params& params) {
    vector<cut*> shifted_cuts = shape_layout_to_cuts(shapes_to_cut, params);
    vector<block> p = gcode_blocks_for_cuts(shifted_cuts, params);
    stringstream ss;
    ss.setf(ios::fixed, ios::floatfield);
    ss.setf(ios::showpoint);
    ss << p << endl;
    return ss.str();
  }
  
}
