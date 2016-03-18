#include <algorithm>
#include <numeric>
#include <sstream>

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
    if (next_cut->tool_no == DRILL) {
      return move_to_next_cut_drill(last_cut, next_cut, params);
    } else if (next_cut->tool_no == DRAG_KNIFE) {
      return move_to_next_cut_dn(last_cut, next_cut, params);
    } else {
      assert(false);
    }
  }

  vector<cut*> insert_transitions(const vector<cut*>& cuts,
				  const cut_params& params) {
    vector<cut*> all_cuts;
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    for (vector<cut*>::const_iterator it = cuts.begin();
	 it != cuts.end(); ++it) {
      next_cut = *it; //cuts[i];
      vector<cut*> transition = move_to_next_cut(last_cut, next_cut, params);
      for (vector<cut*>::iterator jt = transition.begin();
	   jt != transition.end(); ++jt) {
	(*jt)->tool_no = next_cut->tool_no;
      }
      all_cuts.insert(all_cuts.end(), transition.begin(), transition.end());
      all_cuts.push_back(next_cut);
      last_cut = *it; //cuts[i];
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

  vector<cut*> shape_layout_to_cuts(const shape_layout& shapes_to_cut,
				    const cut_params& params) {
    vector<cut*> scuts = shape_cuts(shapes_to_cut, params);
    vector<cut*> all_cuts = insert_transitions(scuts, params);
    assert(cuts_are_adjacent(all_cuts));
    double scale = 1.0;
    point shift(0, 0, params.machine_z_zero);
    vector<cut*> shifted_cuts = shift_and_scale_cuts(all_cuts, shift, scale);
    set_feedrates(shifted_cuts, params);
    return shifted_cuts;
  }

  string shape_layout_to_gcode_string(const shape_layout& shapes_to_cut,
				      const cut_params& params) {
    vector<cut*> shifted_cuts = shape_layout_to_cuts(shapes_to_cut, params);
    vector<block> p = gcode_blocks_for_cuts(shifted_cuts, params);
    stringstream ss;
    ss << p << endl;
    return ss.str();
  }
  
}
