#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/cut_to_gcode.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/shapes_to_toolpaths.h"

using namespace std;

namespace gca {

  void move_to_next_cut_dn(cut* last_cut,
			   cut* next_cut,
			   gprog& p,
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
    if (!within_eps(current_orient, next_orient)) {
      vector<cut*> tcuts = from_to_with_G0_drag_knife(params.safe_height,
						      align_depth,
						      &p,
						      current_loc,
						      current_orient,
						      next_loc,
						      next_orient);
      for (unsigned i = 0; i < tcuts.size(); i++) {
	append_cut(tcuts[i], p, params);
      }      
    } else if (!within_eps(current_loc, next_loc)) {
      vector<cut*> tcuts = from_to_with_G0_height(&p, current_loc, next_cut->start, params.safe_height, mk_lit(params.default_feedrate));
      for (unsigned i = 0; i < tcuts.size(); i++) {
	append_cut(tcuts[i], p, params);
      }      
    }
  }

  void move_to_next_cut_drill(cut* last_cut,
			      cut* next_cut,
			      gprog& p,
			      const cut_params& params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->end;
    
    if (!within_eps(current_loc, next_cut->start)) {
      vector<cut*> tcuts = from_to_with_G0_height(&p, current_loc, next_cut->start, params.safe_height, mk_lit(params.default_feedrate));
      for (unsigned i = 0; i < tcuts.size(); i++) {
	append_cut(tcuts[i], p, params);
      }
    }
  }

  void move_to_next_cut(cut* last_cut,
			cut* next_cut,
			gprog& p,
			const cut_params& params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->end;
    if (next_cut->tool_no == 2) {
      move_to_next_cut_drill(last_cut, next_cut, p, params);
    } else if (next_cut->tool_no == 6) {
      move_to_next_cut_dn(last_cut, next_cut, p, params);
    } else {
      assert(false);
    }
  }

  int get_tool_no(const cut* t) { return t->tool_no; }

  int toolpath_transition(int next, int previous) {
    return previous == next ? -1 : next;
  }

  bool toolpath_is_empty(const toolpath& t) {
    return t.cut_groups.size() == 0;
  }
  
  void append_transition_if_needed(int trans, gprog& p, const cut_params& params) {
    if (trans == -1) {
    } else if (trans == 2) {
      append_drill_header(&p, params.target_machine);
    } else if (trans == 6) {
      append_drag_knife_transfer(&p);
    } else {
      assert(false);
    }
  }

  void append_cuts_gcode(const vector<cut*>& cuts,
			 gprog& p,
			 const cut_params& params) {
    vector<int> active_tools(cuts.size());
    transform(cuts.begin(), cuts.end(), active_tools.begin(), get_tool_no);
    
    vector<int> transitions(active_tools.size());
    adjacent_difference(active_tools.begin(),
    			active_tools.end(),
    			transitions.begin(),
    			toolpath_transition);
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    for (unsigned i = 0; i < cuts.size(); i++) {
      append_transition_if_needed(transitions[i], p, params);
      next_cut = cuts[i];
      move_to_next_cut(last_cut, next_cut, p, params);
      append_cut(next_cut, p, params);
      last_cut = cuts[i];
    }
  }
  
  vector<cut*> flatten_toolpaths(const vector<toolpath>& toolpaths) {
    vector<cut*> cuts;
    for (vector<toolpath>::const_iterator it = toolpaths.begin();
	 it != toolpaths.end(); ++it) {
      const toolpath& t = *it;
      for (vector<cut_group>::const_iterator jt = t.cut_groups.begin();
	   jt != t.cut_groups.end(); ++jt) {
	const cut_group& g = *jt;
	for (cut_group::const_iterator kt = g.begin(); kt != g.end(); ++kt) {
	  cut* cut = *kt;
	  cut->tool_no = t.tool_no;
	  cuts.push_back(cut);
	}
      }
    }
    return cuts;
  }
  
  gprog* shape_layout_to_gcode(const shape_layout& shapes_to_cut,
			       cut_params params) {
    vector<toolpath> toolpaths = cut_toolpaths(shapes_to_cut, params);
    vector<cut*> cuts = flatten_toolpaths(toolpaths);
    gprog* p = mk_gprog();
    append_cuts_gcode(cuts, *p, params);
    gprog* r = append_footer(p, params.target_machine);
    return r;
  }
}
