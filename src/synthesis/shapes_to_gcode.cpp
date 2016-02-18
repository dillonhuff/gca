#include "core/context.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/shapes_to_gcode.h"

using namespace std;

namespace gca {

  bool continuous_cuts(const cut& last_cut,
		       const cut& next_cut,
		       double max_orientation_change) {
    if (!within_eps(last_cut.end, next_cut.start)) {
      return false;
    }
    double theta = angle_between(last_cut.final_orient(), next_cut.initial_orient());
    return theta <= max_orientation_change;
  }
  
  void collect_adjacent_cuts(const vector<cut*>& cuts,
			     vector<cut*>& cut_group,
			     unsigned i,
			     double max_orientation_change) {
    cut_group.push_back(cuts[i]);
    if (cuts.size() == i - 1) {
      return;
    }
    unsigned j = i;
    while (j < cuts.size() - 1) {
      cut* last_cut = cuts[j];
      cut* next_cut = cuts[j + 1];
      if (!continuous_cuts(*last_cut, *next_cut, max_orientation_change)) {
	break;
      }
      cut_group.push_back(cuts[j]);
      j++;
    }
  }

  void group_adjacent_cuts(const vector<cut*>& cuts,
			   vector<cut_group>& cut_groups,
			   double max_orientation_change) {
    unsigned i = 0;
    while (i < cuts.size()) {
      cut_group cut_g;
      collect_adjacent_cuts(cuts, cut_g, i, max_orientation_change);
      assert(cut_g.size() > 0);
      cut_groups.push_back(cut_g);
      i += cut_g.size();
    }
  }
    
  void append_pass_code(gprog* p,
			point current_loc,
			point current_orient,
			const vector<cut*> cut_pass,
			cut_params params) {
    double align_depth = params.material_depth - params.push_depth;
    point next_orient = cut_pass.front()->end - cut_pass.front()->start;
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
      from_to_with_G0_height(p, current_loc, cut_pass.front()->start, params.safe_height);
    }
    for (int j = 0; j < cut_pass.size(); j++) {
      point next_loc = cut_pass[j]->end;
      instr* move_instr = mk_G1(next_loc.x, next_loc.y, next_loc.z, mk_omitted());
      p->push_back(move_instr);
    }
  }

  void make_cut_group_passes(cut_params params,
			     const vector<cut*>& current_group,
			     vector<cut_group>& cut_group_passes) {
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

  // toolpath drag_knife_toolpath(const vector<cut_group>& cut_groups,
  // 			       cut_params params) {
  //   toolpath t;
  //   t.tool_no = 6;

  //   vector<cut_group> cut_group_passes;
  //   for (unsigned j = 0; j < cut_groups.size(); j++) {
  //     vector<cut*> current_group = cut_groups[j];
  //     make_cut_group_passes(params,
  // 			    current_group,
  // 			    cut_group_passes);
  //   }

  //   t.cut_groups = cut_group_passes;
  //   return t;
  // }

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
    from_to_with_G0_height(&p, last_loc, next->start, params.safe_height);
  }

  void add_drill_cuts(const cut_group& cg, gprog& p, const cut_params& params) {
    for (unsigned i = 0; i < cg.size(); i++) {
      cut* ci = cg[i];
      if (ci->is_hole_punch()) {
      } else if (ci->is_linear_cut()) {
	p.push_back(mk_G1(ci->end.x, ci->end.y, ci->end.z));
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
      vector<cut*> cut_pass = cut_passes[i];
      append_pass_code(&p,
		       current_loc,
		       current_orient,
		       cut_pass,
		       params);
      current_loc = cut_pass.back()->end;
      current_orient = cut_pass.back()->end - cut_pass.back()->start;
    }
  }

  gprog* shape_layout_to_gcode(const shape_layout& shapes_to_cut,
			       cut_params params) {
    gprog* p = mk_gprog();
    toolpath dt = drill_toolpath(shapes_to_cut.holes, params);
    if (dt.cut_groups.size() > 0) {
      append_drill_header(p);
      append_drill_toolpath(dt, *p, params);
    }
    vector<cut*> lines_to_cut = shapes_to_cut.lines;
    vector<cut_group> cut_groups;
    append_splines(shapes_to_cut.splines, cut_groups);
    group_adjacent_cuts(lines_to_cut, cut_groups, 30.0);

    if (params.tools == ToolOptions::DRILL_AND_DRAG_KNIFE ||
	params.tools == ToolOptions::DRAG_KNIFE_ONLY) {
      toolpath kt = cut_toolpath(6, cut_groups, params);
      if (kt.cut_groups.size() > 0) {
	append_drag_knife_transfer(p);
	append_drag_knife_toolpath(kt, *p, params);
      }
    } else if (params.tools == ToolOptions::DRILL_ONLY) {
      toolpath drill_cuts = cut_toolpath(2, cut_groups, params);
      if (drill_cuts.cut_groups.size() > 0) {
	append_drill_toolpath(drill_cuts, *p, params);
      }
    } else {
      assert(false);
    }
    gprog* r = append_footer(p);
    return r;
  }

}
