#include <sstream>

#include "analysis/gcode_to_cuts.h"
#include "analysis/machine_state.h"
#include "analysis/position_table.h"
#include "analysis/utils.h"
#include "gcode/lexer.h"
#include "gcode/circular_arc.h"
#include "gcode/circular_helix_cut.h"
#include "gcode/linear_cut.h"
#include "gcode/safe_move.h"
#include "utils/algorithm.h"

namespace gca {

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
	tn = NO_TOOL; //assert(false);
      }
    } else {
      tn = DRILL;
    }
    return tn;
  }

  cut* mk_circular_arc(const machine_state& s, const point cur, const point n) {
    tool_name t = get_tool(s);
    cut* c;
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
      if (s.active_plane != XY_PLANE) {
	cout << "Active plane = " << s.active_plane << endl;
	assert(s.active_plane == XY_PLANE);
      }
      assert(s.k->is_omitted());
      assert(s.i->is_lit() || s.i->is_omitted());
      assert(s.j->is_lit() || s.j->is_omitted());
      double iv = s.i->is_lit() ? static_cast<lit*>(s.i)->v : 0.0;
      double jv = s.j->is_lit() ? static_cast<lit*>(s.j)->v : 0.0;
      point offset(iv, jv, 0);
      if (within_eps(cur.z, n.z)) {
	c = circular_arc::make(cur, n, offset, d, XY);
      } else {
	c = circular_helix_cut::make(cur, n, offset, d, XY);
      }
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

  safe_move* mk_fast_move(const machine_state& s, const point c, const point n) {
    auto tn = get_tool(s);
    safe_move* ct = safe_move::make(c, n, tn);
    ct->set_feedrate(s.feedrate);
    ct->set_spindle_speed(s.spindle_speed);
    return ct;
  }
  
  cut* compute_next_cut(const machine_state& current_state,
			point current_position,
			point last_position) {
    cut* c = nullptr;
    switch (current_state.active_move_type) {
    case FAST_MOVE:
      c = mk_fast_move(current_state, last_position, current_position);
      break;
    case LINEAR_MOVE:
      c = mk_linear_cut(current_state, last_position, current_position);
      break;
    case CLOCKWISE_CIRCULAR_MOVE:
      c = mk_circular_arc(current_state, last_position, current_position);
      break;
    case COUNTERCLOCKWISE_CIRCULAR_MOVE:
      c = mk_circular_arc(current_state, last_position, current_position);
      break;
    default:
      c = nullptr;
    }

    if (c != nullptr) {
      c->settings = extract_settings(current_state);
      c->set_line_number(current_state.line_no);
      // c->set_spindle_speed(current_state.spindle_speed);
      // c->set_feedrate(current_state.feedrate);
      // c->settings.active_tool = current_state.active_tool;
    }

    return c;
  }

  gcode_to_cuts_result fill_paths(const vector<block>& blocks,
				  vector<vector<pair<machine_state, position>>>& paths) {
    auto states = all_program_states(blocks);
    // TODO: Extract this check for correct settings into a new function
    for (auto current_state : states) {
      if (current_state.active_plane == ZX_PLANE ||
    	  current_state.active_plane == YZ_PLANE)
    	{ return GCODE_TO_CUTS_UNSUPPORTED_SETTINGS; }
    }
    vector<vector<machine_state>> toolpaths;
    split_by(states, toolpaths, [](const machine_state& c, const machine_state& p)
	     { return c.active_tool == p.active_tool; });
    for (auto toolpath : toolpaths) {
      coord_system c = toolpath.back().active_coord_system;
      auto ptbl = select_column(c, program_position_table(toolpath));
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
    return GCODE_TO_CUTS_SUCCESS;
  }

  gcode_to_cuts_result gcode_to_cuts(const vector<block>& blocks,
				     vector<vector<cut*>>& cuts) {
    vector<vector<pair<machine_state, position>>> sub_paths;
    auto r = fill_paths(blocks, sub_paths);
    if (r != GCODE_TO_CUTS_SUCCESS) { return r; }
    for (auto path : sub_paths) {
      vector<cut*> cts;
      for (unsigned i = 1; i < path.size(); i++) {
	machine_state current_state = path[i].first;
	point current_position = path[i].second.extract_point();
	point last_position = path[i - 1].second.extract_point();
	if (is_move(current_state)) {
	  cut* next_cut = compute_next_cut(current_state, current_position, last_position);
	  if (next_cut == nullptr)
	    { return GCODE_TO_CUTS_UNSUPPORTED_SETTINGS; }
	  else
	    { cts.push_back(next_cut); }
	}
      }
      if (cts.size() > 0) { cuts.push_back(cts); }
    }
    return GCODE_TO_CUTS_SUCCESS;
  }

  ostream& operator<<(ostream& out, const gcode_to_cuts_result r) {
    switch (r) {
    case GCODE_TO_CUTS_SUCCESS:
      out << "GCODE_TO_CUTS_SUCCESS";
      break;
    case GCODE_TO_CUTS_PATHOLOGICAL_TOOLPATH:
      out << "GCODE_TO_CUTS_PATHOLOGICAL_TOOLPATH";
      break;
    case GCODE_TO_CUTS_UNSUPPORTED_SETTINGS:
      out << "GCODE_TO_CUTS_UNSUPPORTED_SETTINGS";
      break;
    default:
      cout << "Unsupported gcode_to_cuts_result" << endl;
      assert(false);
    }
    return out;
  }
}
