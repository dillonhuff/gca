#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/cut_to_gcode.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/schedule_cuts.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/shapes_to_toolpaths.h"

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
      current_loc = last_cut->end;
      current_orient = last_cut->final_orient();
    }

    double align_depth = params.material_depth - params.push_depth;
    point next_orient = next_cut->initial_orient();
    point next_loc = next_cut->start;
    vector<cut*> tcuts;
    if (!within_eps(angle_between(current_orient, next_orient), 0, params.max_orientation_diff)) {
      tcuts = from_to_with_G0_drag_knife(params.safe_height,
					 align_depth,
					 current_loc,
					 current_orient,
					 next_loc,
					 next_orient);
    } else if (!within_eps(current_loc, next_loc)) {
      tcuts = from_to_with_G0_height(current_loc, next_cut->start, params.safe_height, lit::make(params.default_feedrate));
    }
    return tcuts;
  }

  vector<cut*> move_to_next_cut_drill(cut* last_cut,
				      cut* next_cut,
				      const cut_params& params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->end;

    vector<cut*> tcuts;
    if (!within_eps(current_loc, next_cut->start)) {
      tcuts = from_to_with_G0_height(current_loc, next_cut->start, params.safe_height, lit::make(params.default_feedrate));
    }
    return tcuts;
  }

  vector<cut*> move_to_next_cut(cut* last_cut,
				cut* next_cut,
				const cut_params& params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->end;
    if (next_cut->tool_no == DRILL) {
      return move_to_next_cut_drill(last_cut, next_cut, params);
    } else if (next_cut->tool_no == DRAG_KNIFE) {
      return move_to_next_cut_dn(last_cut, next_cut, params);
    } else {
      assert(false);
    }
  }

  tool_name get_tool_no(const cut* t) { return t->tool_no; }

  tool_name toolpath_transition(tool_name next, tool_name previous) {
    return previous == next ? NO_TOOL : next;
  }

  void append_transition_if_needed(tool_name trans, gprog& p, const cut_params& params) {
    if (trans == NO_TOOL) {
    } else if (trans == DRILL) {
      append_drill_header(&p, params.target_machine);
    } else if (trans == DRAG_KNIFE) {
      append_drag_knife_transfer(&p, params.target_machine);
    } else {
      assert(false);
    }
  }

  void append_cuts_gcode(const vector<cut*>& cuts,
			 gprog& p,
			 const cut_params& params) {
    vector<tool_name> active_tools(cuts.size());
    transform(cuts.begin(), cuts.end(), active_tools.begin(), get_tool_no);
    
    vector<tool_name> transitions(active_tools.size());
    adjacent_difference(active_tools.begin(),
    			active_tools.end(),
    			transitions.begin(),
    			toolpath_transition);

    for (unsigned i = 0; i < cuts.size(); i++) {
      append_transition_if_needed(transitions[i], p, params);
      append_cut(cuts[i], p);
    }
  }
  
  vector<cut*> insert_transitions(const vector<cut*>& cuts,
				  const cut_params& params) {
    vector<cut*> all_cuts;
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    for (unsigned i = 0; i < cuts.size(); i++) {
      next_cut = cuts[i];
      vector<cut*> transition = move_to_next_cut(last_cut, next_cut, params);
      for (unsigned j = 0; j < transition.size(); j++) {
	transition[j]->tool_no = next_cut->tool_no;
      }
      all_cuts.insert(all_cuts.end(), transition.begin(), transition.end());
      all_cuts.push_back(next_cut);
      last_cut = cuts[i];
    }
    return all_cuts;
  }

  bool cuts_are_adjacent(const vector<cut*>& cuts) {
    if (cuts.size() > 0) {
      for (unsigned i = 0; i < cuts.size() - 1; i++) {
	cut* current = cuts[i];
	cut* next = cuts[i+1];
	if (!within_eps(current->end, next->start) &&
	    current->tool_no == next->tool_no) {
	  cout << "Error: " << current->end << " and " << endl;
	  cout << next->start << " are not adjacent" << endl;
	}
      }
    }
    return true;
  }

  vector<cut*> shift_and_scale_cuts(const vector<cut*>& cuts, point p, double s) {
    vector<cut*> shifted_cuts;
    for (vector<cut*>::const_iterator it = cuts.begin();
	 it != cuts.end(); ++it) {
      shifted_cuts.push_back((*it)->shift(p)->scale(s));
    }
    return shifted_cuts;
  }

  void set_feedrates(vector<cut*>& cuts,
		     const cut_params& params) {
    if (params.set_default_feedrate) {
      for (vector<cut*>::iterator it = cuts.begin();
    	     it != cuts.end(); ++it) {
    	(*it)->feedrate = lit::make(params.default_feedrate);
      }
    }    
  }

  gprog* shape_layout_to_gcode(const shape_layout& shapes_to_cut,
			       cut_params params) {
    vector<cut*> scuts = shape_cuts(shapes_to_cut, params);
    vector<cut*> cuts = schedule_cuts(scuts);
    vector<cut*> all_cuts = insert_transitions(cuts, params);
    assert(cuts_are_adjacent(all_cuts));
    double scale = 1.0;
    point shift(0, 0, params.machine_z_zero);
    vector<cut*> shifted_cuts = shift_and_scale_cuts(all_cuts, shift, scale);
    set_feedrates(shifted_cuts, params);
    gprog* p = gprog::make();
    append_cuts_gcode(shifted_cuts, *p, params);
    gprog* r = append_footer(p, params.target_machine);
    return r;
  }

}
