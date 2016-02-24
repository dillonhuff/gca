#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/shapes_to_toolpaths.h"

using namespace std;

namespace gca {

  void append_cuts(const cut_group& cg, gprog& p, const cut_params& params) {
    for (unsigned i = 0; i < cg.size(); i++) {
      cut* ci = cg[i];
      if (ci->is_hole_punch()) {
      } else if (ci->is_linear_cut()) {
	p.push_back(mk_G1(ci->end.x, ci->end.y, ci->end.z, params.default_feedrate));
      } else if (ci->is_circular_arc()) {
	circular_arc* arc = static_cast<circular_arc*>(ci);
	if (arc->dir == CLOCKWISE) {
	  p.push_back(mk_G2(mk_lit(arc->end.x),
			    mk_lit(arc->end.y),
			    mk_lit(params.pass_depth),
			    mk_lit(arc->start_offset.x),
			    mk_lit(arc->start_offset.y),
			    mk_lit(arc->start_offset.z),
			    mk_lit(params.default_feedrate)));
	} else if (arc->dir == COUNTERCLOCKWISE) {
	  p.push_back(mk_G3(mk_lit(arc->end.x),
			    mk_lit(arc->end.y),
			    mk_lit(params.pass_depth),
			    mk_lit(arc->start_offset.x),
			    mk_lit(arc->start_offset.y),
			    mk_lit(arc->start_offset.z),
			    mk_lit(params.default_feedrate)));
	} else {
	  assert(false);
	}
      } else {
	assert(false);
      }
    }
  }
  
  void move_to_next_cut_dn(cut* last_cut,
			   cut* next_cut,
			   gprog& p,
			   cut_params params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->end;
    point current_orient = last_cut == NULL ? params.start_orient : last_cut->final_orient();

    double align_depth = params.material_depth - params.push_depth;
    point next_orient = next_cut->initial_orient();
    point next_loc = next_cut->start;
    if (!within_eps(current_orient, next_orient)) {
      from_to_with_G0_drag_knife(params.safe_height,
      				 align_depth,
      				 &p,
      				 current_loc,
      				 current_orient,
      				 next_loc,
      				 next_orient);
    } else {
      from_to_with_G0_height(&p, current_loc, next_cut->start, params.safe_height, mk_lit(params.default_feedrate));
    }
  }

  void move_to_next_cut_drill(cut* last_cut,
			      cut* next_cut,
			      gprog& p,
			      const cut_params& params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->end;
    
    if (!within_eps(current_loc, next_cut->start)) {
      from_to_with_G0_height(&p, current_loc, next_cut->start, params.safe_height,
      			     mk_lit(params.default_feedrate));
    }
  }

  void move_to_next_cut(int tool_no,
			cut* last_cut,
			cut* next_cut,
			gprog& p,
			const cut_params& params) {
    point current_loc = last_cut == NULL ? params.start_loc : last_cut->end;
    if (tool_no == 2) {
      move_to_next_cut_drill(last_cut, next_cut, p, params);
    } else if (tool_no == 6) {
      move_to_next_cut_dn(last_cut, next_cut, p, params);
    } else {
      assert(false);
    }
  }

  int get_tool_no(const toolpath& t) { return t.tool_no; }

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

  void append_toolpath_code(const toolpath& t, gprog& p, const cut_params& params) {
    const vector<cut_group>& cgs = t.cut_groups;
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    for (unsigned i = 0; i < cgs.size(); i++) {
      cut_group cg = cgs[i];
      next_cut = cg.front();
      move_to_next_cut(t.tool_no, last_cut, next_cut, p, params);
      append_cuts(cg, p, params);
      last_cut = cg.back();
    }
  }

  void append_toolpaths(vector<toolpath>& toolpaths,
			gprog& p,
			const cut_params& params) {
    toolpaths.erase(remove_if(toolpaths.begin(), toolpaths.end(), toolpath_is_empty),
    		    toolpaths.end());
    vector<int> active_tools(toolpaths.size());
    transform(toolpaths.begin(), toolpaths.end(), active_tools.begin(), get_tool_no);
    
    vector<int> transitions(active_tools.size());
    adjacent_difference(active_tools.begin(),
    			active_tools.end(),
    			transitions.begin(),
    			toolpath_transition);

    for (unsigned i = 0; i < toolpaths.size(); i++) {
      append_transition_if_needed(transitions[i], p, params);
      append_toolpath_code(toolpaths[i], p, params);
    }
  }
  
  gprog* shape_layout_to_gcode(const shape_layout& shapes_to_cut,
			       cut_params params) {
    vector<toolpath> toolpaths = cut_toolpaths(shapes_to_cut, params);
    gprog* p = mk_gprog();
    append_toolpaths(toolpaths, *p, params);
    gprog* r = append_footer(p, params.target_machine);
    return r;
  }
}
