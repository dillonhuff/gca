#include "core/context.h"
#include "synthesis/dxf_reader.h"
#include "synthesis/dxf_to_gcode.h"

using namespace std;

namespace gca {
  
  typedef vector<cut*> cut_group;

  void collect_adjacent_cuts(const vector<cut*>& cuts,
			     vector<cut*>& cut_group,
			     unsigned i) {
    cut_group.push_back(cuts[i]);
    if (cuts.size() == i - 1) {
      return;
    }
    unsigned j = i;
    while (j < cuts.size() - 1) {
      point last_cut_end = cuts[j]->end;
      point next_cut_start = cuts[j + 1]->start;
      if (!within_eps(last_cut_end, next_cut_start)) {
	break;
      }
      cut_group.push_back(cuts[j]);
      j++;
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

  void append_cut_group_code(gprog* p,
			     const vector<cut_group> cut_passes,
			     cut_params params) {
    point current_loc = params.start_loc;
    point current_orient = params.start_orient;
    for (unsigned i = 0; i < cut_passes.size(); i++) {
      vector<cut*> cut_pass = cut_passes[i];
      append_pass_code(p,
		       current_loc,
		       current_orient,
		       cut_pass,
		       params);
      current_loc = cut_pass.back()->end;
      current_orient = cut_pass.back()->end - cut_pass.back()->start;
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

  void append_cut_code(const dxf_reader& l, gprog* p, cut_params params) {
    vector<cut_group> cut_groups;
    append_splines(l.splines, cut_groups);
    unsigned i = 0;
    cout << "Num cuts: " << l.cuts.size() << endl;
    while (i < l.cuts.size()) {
      cut_group cut_g;
      collect_adjacent_cuts(l.cuts, cut_g, i);
      assert(cut_g.size() > 0);
      cut_groups.push_back(cut_g);
      i += cut_g.size();
    }
    cout << "Num cut groups = " << cut_groups.size() << endl;
    vector<cut_group> cut_group_passes;
    for (unsigned j = 0; j < cut_groups.size(); j++) {
      vector<cut*> current_group = cut_groups[j];
      make_cut_group_passes(params,
			    current_group,
			    cut_group_passes);
    }
    append_cut_group_code(p, cut_group_passes, params);
  }

  void append_drill_code(const vector<hole_punch*>& punches,
			 gprog* p,
			 cut_params params) {
    point current_loc = params.start_loc;
    for (unsigned i = 0; i < punches.size(); i++) {
      hole_punch* punch = punches[i];
      point next_loc = punch->start;
      from_to_with_G0_height(p,
			     current_loc,
			     next_loc,
			     params.safe_height);
      current_loc = punch->end;
    }
  }

  gprog* dxf_to_gcode(char* file, cut_params params) {
    dxf_reader* listener = new dxf_reader();
    DL_Dxf* dxf = new DL_Dxf();
    if (!dxf->in(file, listener)) {
      std::cerr << file << " could not be opened.\n";
      return NULL;
    }
    delete dxf;
    gprog* p = mk_gprog();
    append_drill_header(p);
    append_drill_code(listener->hole_punches, p, params);
    append_drag_knife_transfer(p);
    append_cut_code(*listener, p, params);
    gprog* r = append_footer(p);
    delete listener;
    return r;
  }

}
