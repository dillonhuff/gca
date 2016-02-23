#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/shapes_to_gcode.h"

using namespace std;

namespace gca {
    
  void append_pass_code(gprog* p,
			point current_loc,
			point current_orient,
			const vector<cut*>& cut_pass,
			cut_params params) {
    double align_depth = params.material_depth - params.push_depth;
    point next_orient = cut_pass.front()->initial_orient();
    point next_loc = cut_pass.front()->start;
    if (!within_eps(current_orient, next_orient)) {
      from_to_with_G0_drag_knife(params.safe_height,
				 align_depth,
				 p,
				 current_loc,
				 current_orient,
				 next_loc,
				 next_orient);
    } else {
      from_to_with_G0_height(p, current_loc, cut_pass.front()->start, params.safe_height, mk_lit(params.default_feedrate));
    }
    for (unsigned j = 0; j < cut_pass.size(); j++) {
      point next_loc = cut_pass[j]->end;
      instr* move_instr = mk_G1(next_loc.x, next_loc.y, next_loc.z, params.default_feedrate);
      p->push_back(move_instr);
    }
  }

  void make_cut_group_passes(cut_params params,
			     const vector<cut*>& current_group,
			     vector<cut_group>& cut_group_passes) {
    if  (params.one_pass_only) {
      vector<cut*> new_pass;
      for (unsigned i = 0; i < current_group.size(); i++) {
	cut* ct = current_group[i];
	if (ct->is_linear_cut()) {
	  new_pass.push_back(mk_linear_cut(point(ct->start.x, ct->start.y, params.pass_depth), point(ct->end.x, ct->end.y, params.pass_depth)));
	} else if (ct->is_circular_arc()) {
	  circular_arc* arc = static_cast<circular_arc*>(ct);
	  point s = arc->start;
	  s.z = params.pass_depth;
	  point e = arc->end;
	  e.z = params.pass_depth;
	  new_pass.push_back(circular_arc::make(s, e, arc->start_offset, arc->dir, arc->pl));
	} else {
	  assert(false);
	}
      }
      cut_group_passes.push_back(new_pass);
      
    } else {
      assert(params.cut_depth < params.material_depth);
      double depth = params.material_depth - params.cut_depth;
      while (true) {
	vector<cut*> new_pass;
	for (unsigned i = 0; i < current_group.size(); i++) {
	  cut* ct = current_group[i];
	  assert(ct->is_linear_cut());
	  new_pass.push_back(mk_linear_cut(point(ct->start.x, ct->start.y, depth), point(ct->end.x, ct->end.y, depth)));
	}
	cut_group_passes.push_back(new_pass);
	if (depth == 0.0) {
	  break;
	}
	depth = max(0.0, depth - params.cut_depth);
      }
    }
  }

  void append_spline(const b_spline* s,
		     vector<vector<cut*> >& cut_groups) {
    unsigned points_per_spline = 100;
    vector<cut*> spline_cuts;
    double last = 0.0;
    double inc = 1.0 / points_per_spline;
    for (unsigned i = 1; i < points_per_spline; i++) {
      double next = last + inc;
      spline_cuts.push_back(mk_linear_cut(s->eval(last), s->eval(next)));
      last = next;
    }
    cut_groups.push_back(spline_cuts);
  }

  void append_splines(const vector<b_spline*>& splines,
		      vector<cut_group>& cut_groups) {
    for (unsigned i = 0; i < splines.size(); i++) {
      append_spline(splines[i], cut_groups);
    }
  }

  toolpath drill_toolpath(const vector<hole_punch*>& holes,
			  cut_params params) {
    toolpath t;
    t.tool_no = 2;
    vector<cut_group> punch_groups;
    for (unsigned i = 0; i < holes.size(); i++) {
      cut_group one_hole;
      one_hole.push_back(holes[i]);
      punch_groups.push_back(one_hole);
    }
    t.cut_groups = punch_groups;
    return t;
  }

  toolpath cut_toolpath(int tool_number,
			const vector<cut_group>& cut_groups,
			cut_params params) {
    toolpath t;
    t.tool_no = tool_number;
    vector<cut_group> cut_group_passes;
    for (unsigned j = 0; j < cut_groups.size(); j++) {
      vector<cut*> current_group = cut_groups[j];
      make_cut_group_passes(params,
			    current_group,
			    cut_group_passes);
    }

    t.cut_groups = cut_group_passes;
    return t;
  }

  void move_to_next_cut(cut* last,
			cut* next,
			gprog& p,
			const cut_params& params) {
    point last_loc;
    if (last == NULL) {
      last_loc = params.start_loc;
    } else {
      last_loc = last->end;
    }
    if (!within_eps(last_loc, next->start)) {
      from_to_with_G0_height(&p, last_loc, next->start, params.safe_height,
			     mk_lit(params.default_feedrate));
    }
  }

  void add_drill_cuts(const cut_group& cg, gprog& p, const cut_params& params) {
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

  void append_drill_toolpath(const toolpath& t, gprog& p, cut_params params) {
    const vector<cut_group>& cgs = t.cut_groups;
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    for (unsigned i = 0; i < cgs.size(); i++) {
      cut_group cg = cgs[i];
      next_cut = cg.front();
      move_to_next_cut(last_cut, next_cut, p, params);
      add_drill_cuts(cg, p, params);
      last_cut = cg.back();
    }
  }

  void append_drag_knife_toolpath(const toolpath& t, gprog& p, cut_params params) {
    point current_loc = params.start_loc;
    point current_orient = params.start_orient;
    vector<cut_group> cut_passes = t.cut_groups;
    for (unsigned i = 0; i < cut_passes.size(); i++) {
      cut_group cut_pass = cut_passes[i];
      append_pass_code(&p,
		       current_loc,
		       current_orient,
		       cut_pass,
		       params);
      current_loc = cut_pass.back()->end;
      current_orient = cut_pass.back()->end - cut_pass.back()->start;
    }
  }

  void create_toolpaths(const shape_layout& shapes_to_cut,
			vector<toolpath>& toolpaths,
			const cut_params& params) {
    if (params.tools != DRAG_KNIFE_ONLY) {
      toolpaths.push_back(drill_toolpath(shapes_to_cut.holes, params));
    }

    vector<cut_group> cut_groups;
    append_splines(shapes_to_cut.splines, cut_groups);
    group_adjacent_cuts(shapes_to_cut.lines, cut_groups, 30.0);

    if (params.tools == DRILL_AND_DRAG_KNIFE ||
	params.tools == DRAG_KNIFE_ONLY) {
      toolpaths.push_back(cut_toolpath(6, cut_groups, params));
    } else if (params.tools == DRILL_ONLY) {
      toolpaths.push_back(cut_toolpath(2, cut_groups, params));
    } else {
      assert(false);
    }
  }

  int get_tool_no(const toolpath& t) { return t.tool_no; }

  int toolpath_transition(int next, int previous) {
    return previous == next ? -1 : next;
  }

  int cmp_tools(const toolpath& l, const toolpath& r) {
    return l.tool_no < r.tool_no;
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
    if (t.tool_no == 2) {
      append_drill_toolpath(t, p, params);
    } else if (t.tool_no == 6) {
      append_drag_knife_toolpath(t, p, params);
    } else {
      assert(false);
    }
  }

  void append_toolpaths(vector<toolpath>& toolpaths,
			gprog& p,
			const cut_params& params) {
    toolpaths.erase(remove_if(toolpaths.begin(), toolpaths.end(), toolpath_is_empty),
		    toolpaths.end());
    stable_sort(toolpaths.begin(), toolpaths.end(), cmp_tools);
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
    vector<toolpath> toolpaths;
    create_toolpaths(shapes_to_cut, toolpaths, params);
    gprog* p = mk_gprog();
    append_toolpaths(toolpaths, *p, params);
    gprog* r = append_footer(p, params.target_machine);
    return r;
  }
}
