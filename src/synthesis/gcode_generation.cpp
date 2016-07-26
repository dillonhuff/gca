#include "gcode/linear_cut.h"
#include "synthesis/cut_to_gcode.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/shape_layout.h"
#include "synthesis/shapes_to_gcode.h"
#include "utils/algorithm.h"

namespace gca {

  std::vector<polyline> reflect_y(const std::vector<polyline>& pocket_lines) {
    vector<polyline> reflected;
    for (auto p : pocket_lines) {
      reflected.push_back(p.apply([](const point p)
				  { return point(p.x, -p.y, p.z); }));
    }
    return reflected;
  }
  
  // TODO: Make the spindle_speed and feedrate parameters explicit
  cut* mk_cut(const point l,
	      const point r,
	      const double spindle_speed,
	      const double feedrate) {
    auto c = linear_cut::make(l, r);
    c->set_spindle_speed(lit::make(spindle_speed));
    c->set_feedrate(lit::make(feedrate));
    return c;
  }

  vector<cut*> polyline_cuts(const polyline& p,
			     const double spindle_speed,
			     const double feedrate) {
    auto ls = p.lines();
    assert(ls.size() > 0);
    vector<cut*> c;
    for (auto l : ls) {
      c.push_back(mk_cut(l.start, l.end, spindle_speed, feedrate));
    }
    return c;
  }

  void append_toolpath_header_blocks(vector<block>& bs, const machine_name m) {
    block b;
    b.push_back(token('G', 90));
    bs.push_back(b);
  }
  
  std::vector<block>
  gcode_blocks_for_toolpath_cuts(const std::vector<cut*>& cuts,
				 const cut_params& params) {
    vector<block> bs;
    append_toolpath_header_blocks(bs, params.target_machine);
    append_cuts_gcode_blocks(cuts, bs, params);
    return bs;
  }

  std::vector<block> polylines_cuts(const vector<polyline>& pocket_lines,
				    const cut_params params,
				    const double spindle_speed,
				    const double feedrate) {
    vector<cut*> cuts;
    for (auto p : pocket_lines) {
      auto cs = polyline_cuts(p, spindle_speed, feedrate);
      cuts.insert(cuts.end(), cs.begin(), cs.end());
    }

    vector<cut*> all_cuts = insert_transitions(cuts, params);
    assert(cuts_are_adjacent(all_cuts));
    set_feedrates(all_cuts, params);
    return gcode_blocks_for_toolpath_cuts(all_cuts, params);
  }

  std::vector<block>
  tool_comment_prefix(const tool& t) {
    vector<block> blks;
    blks.push_back({token("TOOL DIAMETER = " + std::to_string(t.diameter()))});
    blks.push_back({token("TOOL LENGTH = " + std::to_string(t.length()))});
    return blks;
  }
  
  std::vector<block>
  comment_prefix(const toolpath& tp, const cut_params& params) {
    vector<block> blks;
    blks.push_back({token("TOOLPATH")});
    concat(blks, tool_comment_prefix(tp.t));
    return blks;
  }

  std::vector<block> emco_f1_code(const toolpath& tp) {
    for (auto l : tp.lines) {
      assert(l.num_points() > 0);
    }

    point shift_vector = point(0, 0, tp.t.length());
    vector<polyline> reflected_lines =
      shift_lines(reflect_y(tp.lines), shift_vector);

    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = tp.safe_z_before_tlc + tp.t.length();

    vector<block> blks = comment_prefix(tp, params);
    concat(blks, polylines_cuts(reflected_lines, params, tp.spindle_speed, tp.feedrate));
    return blks;
  }

}
