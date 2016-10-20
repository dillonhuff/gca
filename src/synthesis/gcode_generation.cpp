#include <sstream>

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
	      const int tool_num,
	      const double spindle_speed,
	      const double feedrate) {
    auto c = linear_cut::make(l, r);
    c->set_spindle_speed(lit::make(spindle_speed));
    c->set_feedrate(lit::make(feedrate));
    if (tool_num != -1) {
      c->set_tool_number(ilit::make(tool_num));
    }
    return c;
  }

  vector<cut*> polyline_cuts(const polyline& p,
			     const int tool_number,
			     const double spindle_speed,
			     const double feedrate) {
    auto ls = p.lines();
    DBG_ASSERT(ls.size() > 0);
    vector<cut*> c;
    for (auto l : ls) {
      c.push_back(mk_cut(l.start, l.end, tool_number, spindle_speed, feedrate));
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
				    const int tool_number,
				    const cut_params params,
				    const double spindle_speed,
				    const double feedrate) {
    vector<cut*> cuts;
    for (auto p : pocket_lines) {
      auto cs = polyline_cuts(p, tool_number, spindle_speed, feedrate);
      cuts.insert(cuts.end(), cs.begin(), cs.end());
    }

    vector<cut*> all_cuts = insert_transitions(cuts, params);
    DBG_ASSERT(cuts_are_adjacent(all_cuts));
    set_feedrates(all_cuts, params);
    return gcode_blocks_for_toolpath_cuts(all_cuts, params);
  }

  std::vector<block>
  tool_comment_prefix(const toolpath& t, const token_type comment_style) {
    vector<block> blks;
    std::stringstream ss;
    ss << t.pocket_type();
    blks.push_back({token("OPERATION = " + ss.str(), comment_style)});
    blks.push_back({token("TOOL DIAMETER = " + std::to_string(t.t.diameter()),
			  comment_style)});
    blks.push_back({token("TOOL LENGTH = " + std::to_string(t.t.length()),
			  comment_style)});
    blks.push_back({token("TOOL NUMBER = " + std::to_string(t.t.tool_number()),
			  comment_style)});

    return blks;
  }

  std::vector<block>
  g10_TLC_prefix(const toolpath& t) {
    vector<block> blks;

    blks.push_back({token('G', 54)});
    blks.push_back({token('G', 10), token('L', 2), token('P', 1), token('Z', t.t.length())});

    return blks;
  }
  
  std::vector<block>
  comment_prefix(const toolpath& tp, const cut_params& params) {
    vector<block> blks;
    blks.push_back({token("TOOLPATH")});
    concat(blks, tool_comment_prefix(tp, PAREN_COMMENT));
    return blks;
  }

  std::vector<block> camaster_prefix_blocks(const toolpath& intial) {
    vector<block> blks;
    blks.push_back({token('G', 90)});
    blks.push_back({token('G', 53), token('Z', 0.0)});
    blks.push_back({token('M', 5)});

    return blks;
  }
  
  std::vector<block> camaster_suffix_blocks() {
    vector<block> blks;
    blks.push_back({token('G', 53), token('Z', 0.0)});
    blks.push_back({token('M', 5)});
    blks.push_back({token('M', 2)});

    return blks;
  }
  
  std::vector<block>
  camaster_comment_prefix(const toolpath& tp, const cut_params& params) {
    vector<block> blks;
    blks.push_back({token("TOOLPATH", BRACKET_COMMENT)});
    concat(blks, tool_comment_prefix(tp, BRACKET_COMMENT));
    return blks;
  }

  std::vector<block> camaster_engraving(const toolpath& last,
					const toolpath& tp) {
    for (auto l : tp.lines) {
      DBG_ASSERT(l.num_points() > 0);
    }

    cut_params params;
    params.target_machine = CAMASTER;
    params.safe_height = tp.safe_z_before_tlc;
    params.set_plunge_feed(tp.plunge_feedrate);

    vector<block> blks = camaster_comment_prefix(tp, params);
    concat(blks, camaster_tool_change_block(tp.tool_number()));
    concat(blks, polylines_cuts(tp.lines, tp.tool_number(), params, tp.spindle_speed, tp.feedrate));
    return blks;
  }

  std::vector<block> emco_f1_code(const toolpath& tp) {
    for (auto l : tp.lines) {
      DBG_ASSERT(l.num_points() > 0);
    }

    point shift_vector = point(0, 0, tp.t.length());
    vector<polyline> reflected_lines =
      shift_lines(reflect_y(tp.lines), shift_vector);

    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = tp.safe_z_before_tlc + tp.t.length();
    params.set_plunge_feed(tp.plunge_feedrate);

    vector<block> blks = comment_prefix(tp, params);
    concat(blks, polylines_cuts(reflected_lines, tp.tool_number(), params, tp.spindle_speed, tp.feedrate));
    return blks;
  }

  std::vector<block> emco_f1_code_G10_TLC(const toolpath& tp) {
    for (auto l : tp.lines) {
      DBG_ASSERT(l.num_points() > 0);
    }

    vector<polyline> reflected_lines = reflect_y(tp.lines);

    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = tp.safe_z_before_tlc;
    params.set_plunge_feed(tp.plunge_feedrate);

    vector<block> blks = comment_prefix(tp, params);
    concat(blks, g10_TLC_prefix(tp));
    concat(blks, polylines_cuts(reflected_lines, tp.tool_number(), params, tp.spindle_speed, tp.feedrate));
    return blks;
  }

  std::vector<block> emco_f1_code_no_TLC(const toolpath& tp) {
    for (auto l : tp.lines) {
      DBG_ASSERT(l.num_points() > 0);
    }

    vector<polyline> reflected_lines = reflect_y(tp.lines);

    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = tp.safe_z_before_tlc;
    params.set_plunge_feed(tp.plunge_feedrate);

    vector<block> blks = comment_prefix(tp, params);
    concat(blks, polylines_cuts(reflected_lines, tp.tool_number(), params, tp.spindle_speed, tp.feedrate));
    return blks;
  }
  
}
