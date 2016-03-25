#include <sstream>

#include "analysis/gcode_to_cuts.h"
#include "analysis/machine_state.h"
#include "analysis/position_table.h"
#include "analysis/utils.h"
#include "core/lexer.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/safe_move.h"
#include "system/algorithm.h"

namespace gca {

  bool drill_with_spindle_off(const cut* c) {
    return !(!(c->tool_no == DRILL) || (!c->get_spindle_speed()->is_omitted()));
  }

  void sanity_check_speeds(const vector<cut*>& cuts) {
    assert(count_if(cuts.begin(), cuts.end(), drill_with_spindle_off) == 0);
  }

  tool_name get_tool(const machine_state& s) {
    auto t = s.active_tool;
    tool_name tn;
    if (t->is_ilit()) {
      int i = static_cast<ilit*>(t)->v;
      if (i == 6) {
	tn = DRAG_KNIFE;
      } else if (i == 2) {
	tn = DRILL;
      } else {
	assert(false);
      }
    } else {
      tn = DRILL;
    }
    return tn;
  }

  circular_arc* mk_circular_arc(const machine_state& s, const point cur, const point n) {
    tool_name t = get_tool(s);
    circular_arc* c;
    point offset;
    direction d;
    if (s.active_move_type == CLOCKWISE_CIRCULAR_MOVE) {
      d = CLOCKWISE;
    } else if (s.active_move_type == COUNTERCLOCKWISE_CIRCULAR_MOVE) {
      d = COUNTERCLOCKWISE;
    } else {
      assert(false);
    }
    if (!s.k->is_lit()) {
      assert(s.i->is_lit());
      assert(s.j->is_lit());
      double iv = static_cast<lit*>(s.i)->v;
      double jv = static_cast<lit*>(s.j)->v;
      point offset(iv, jv, 0);
      c = circular_arc::make(cur, n, offset, d, XY);
      c->set_spindle_speed(s.spindle_speed);
      c->set_feedrate(s.feedrate);
      c->tool_no = t;
    } else {
      assert(false);
    }
    return c;
  }

  linear_cut* mk_linear_cut(const machine_state& s, const point c, const point n) {
    auto tn = get_tool(s);
    linear_cut* ct = linear_cut::make(c, n, tn);
    ct->set_feedrate(s.feedrate);
    ct->set_spindle_speed(s.spindle_speed);
    return ct;
  }

  pair<vector<machine_state>, vector<point>> clipped_states(const vector<block>& blocks) {
    auto ms = all_program_states(blocks);
    auto ptbl = program_position_table(ms);
    auto positions = select_column(UNKNOWN_COORD_SYSTEM, ptbl);
    unsigned i = 0;
    while (i < positions.size() && !positions[i].is_lit()) { i++; }
    vector<machine_state> clipped_states(ms.begin() + i, ms.end());
    vector<point> clipped_points;
    for (; i < positions.size(); i++) {
      clipped_points.push_back(positions[i].extract_point());
    }
    assert(clipped_points.size() == clipped_states.size());
    return pair<vector<machine_state>, vector<point>>(clipped_states, clipped_points);
  }

  cut* compute_next_cut(const machine_state& current_state,
			point current_position,
			point last_position) {
    switch (current_state.active_move_type) {
    case LINEAR_MOVE:
      return mk_linear_cut(current_state, last_position, current_position);
    case CLOCKWISE_CIRCULAR_MOVE:
      return mk_circular_arc(current_state, last_position, current_position);
    case COUNTERCLOCKWISE_CIRCULAR_MOVE:
      return mk_circular_arc(current_state, last_position, current_position);
    default:
      return nullptr;
    }
  }

  void fill_paths(const vector<block>& blocks,
		  vector<vector<pair<machine_state, position>>>& paths) {
    auto states = all_program_states(blocks);
    vector<vector<machine_state>> toolpaths;
    split_by(states, toolpaths, [](const machine_state& c, const machine_state& p)
	     { return c.active_tool == p.active_tool; });
    for (auto toolpath : toolpaths) {
      auto ptbl = select_column(UNKNOWN_COORD_SYSTEM, program_position_table(toolpath));
      vector<pair<machine_state, position>> tstates(ptbl.size());
      zip(toolpath.begin(), toolpath.end(), ptbl.begin(), tstates.begin());
      vector<vector<pair<machine_state, position>>> sub_paths;
      split_by(tstates, sub_paths,
	       [](const pair<machine_state, position>& x,
		  const pair<machine_state, position>& y)
	       { return x.second.is_lit() == y.second.is_lit(); });
      delete_if(sub_paths, [](const vector<pair<machine_state, position>>& p)
		{ return !p.front().second.is_lit(); });
      paths.insert(end(paths), begin(sub_paths), end(sub_paths));
    }
  }

  gcode_to_cuts_result gcode_to_cuts(const vector<block>& blocks, vector<vector<cut*>>& cuts) {
    vector<vector<pair<machine_state, position>>> sub_paths;
    fill_paths(blocks, sub_paths);
    for (auto path : sub_paths) {
      vector<cut*> cts;
      for (unsigned i = 1; i < path.size(); i++) {
	machine_state current_state = path[i].first;
	point current_position = path[i].second.extract_point();
	point last_position = path[i - 1].second.extract_point();
	cut* next_cut = compute_next_cut(current_state, current_position, last_position);
	if (next_cut != nullptr) { cts.push_back(next_cut); }
      }
      if (cts.size() > 0) { cuts.push_back(cts); }
    }
    return GCODE_TO_CUTS_SUCCESS;
  }
    
}
