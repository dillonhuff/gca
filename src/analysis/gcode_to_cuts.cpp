#include <sstream>

#include "analysis/gcode_to_cuts.h"
#include "analysis/machine_state.h"
#include "analysis/position_table.h"
#include "analysis/utils.h"
#include "core/basic_states.h"
#include "core/lexer.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/safe_move.h"
#include "system/algorithm.h"

namespace gca {

  bool drill_with_spindle_off(const cut* c) {
    return !(!(c->tool_no == DRILL) || (!c->spindle_speed->is_omitted()));
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
      c->spindle_speed = s.spindle_speed;
      c->feedrate = s.feedrate;
      c->tool_no = t;
    } else {
      assert(false);
    }
    return c;
  }

  linear_cut* mk_linear_cut(const machine_state& s, const point c, const point n) {
    auto tn = get_tool(s);
    linear_cut* ct = linear_cut::make(c, n, tn);
    ct->feedrate = s.feedrate;
    ct->spindle_speed = s.spindle_speed;
    return ct;
  }

  pair<vector<machine_state>, vector<point>> clipped_states(const vector<block>& blocks) {
    auto ms = all_program_states(blocks);
    auto ptbl = program_position_table(ms);
    auto positions = select_column(UNKNOWN_COORD_SYSTEM, ptbl);
    unsigned i = 0;
    while (i < positions.size() && !positions[i].is_lit()) { i++; }
    vector<machine_state> clipped_states(ms.begin() + i + 1, ms.end());
    vector<point> clipped_points;
    for (; i < positions.size(); i++) {
      clipped_points.push_back(positions[i].extract_point());
    }
    assert(clipped_points.size() == clipped_states.size());
    return pair<vector<machine_state>, vector<point>>(clipped_states, clipped_points);
  }

  vector<cut*> gcode_to_cuts(const vector<block>& blocks) {
    vector<cut*> cuts;
    auto cs = clipped_states(blocks);
    cout << "Got clipped states" << endl;
    cout << "states: " << endl;
    cout << cs.first << endl;
    cout << "points: " << endl;
    cout << cs.second << endl;
    for (unsigned i = 1; i < cs.first.size(); i++) {
      machine_state current_state = cs.first[i];
      point current_position = cs.second[i];
      point last_position = cs.second[i - 1];
      switch (current_state.active_move_type) {
      case LINEAR_MOVE:
	cuts.push_back(mk_linear_cut(current_state, last_position, current_position));
	break;
      case CLOCKWISE_CIRCULAR_MOVE:
	cuts.push_back(mk_circular_arc(current_state, last_position, current_position));
      case COUNTERCLOCKWISE_CIRCULAR_MOVE:
	cuts.push_back(mk_circular_arc(current_state, last_position, current_position));
	break;
      default:
	break;
      }
    }
    return cuts;
  }

  vector<cut*> gcode_to_cuts(gprog& p, const gcode_settings& settings) {
    stringstream s;
    s << p;
    auto ws = lex_gprog(s.str());
    return gcode_to_cuts(ws);
  }

  bool gcode_settings::sanity_check() const {
    return initial_tool == DRILL || initial_tool == DRAG_KNIFE;
  }
  
}
