#include "analysis/utils.h"
#include "analysis/position_table.h"
#include "system/algorithm.h"

namespace gca {
  
  bool is_end_code(const token t) {
    if (t.tp() == ICODE) {
      return t.c == 'M' && (*(t.v) == ilit(2) || *(t.v) == ilit(30));
    }
    return false;
  }

  bool is_end_block(const block& t) {
    int num_end_codes = count_if(t.begin(), t.end(), is_end_code);
    assert(num_end_codes == 0 || num_end_codes == 1);
    bool contains_end_instr = num_end_codes == 1;
    return contains_end_instr;
  }

  bool is_ret_code(const token t) {
    if (t.tp() == ICODE)
      { return t.c == 'M' && (*(t.v) == ilit(99)); }
    return false;
  }

  bool is_ret_block(const block& t) {
    int num_end_codes = count_if(t.begin(), t.end(), is_ret_code);
    assert(num_end_codes == 0 || num_end_codes == 1);
    bool contains_end_instr = num_end_codes == 1;
    return contains_end_instr;
  }

  bool is_call_code(const token t) {
    if (t.tp() == ICODE) {
      return t.c == 'M' && (*(t.v) == ilit(97));
    }
    return false;
  }
  
  bool is_call_block(const block& t) {
    int num_end_codes = count_if(t.begin(), t.end(), is_call_code);
    assert(num_end_codes == 0 || num_end_codes == 1);
    bool contains_end_instr = num_end_codes == 1;
    return contains_end_instr;
  }

  typedef vector<block>::const_iterator program_loc;
  
  vector<block>::const_iterator
  find_called_subroutine(const block& b,
			 const vector<pair<token, program_loc>>& p) {
    token ic = *find_if(b.begin(), b.end(), is_register('P'));
    token line_no('N', ic.v);
    for (const auto pr : p) {
      if (pr.first == line_no) { return pr.second; }
    }
    assert(false);
  }

  program_loc find_loc(const vector<block>& p, const token& t) {
    return find_if(p.begin(), p.end(), block_contains(t));
  }

  vector<pair<token, program_loc> >
  compute_starts(const vector<block>& p) {
    vector<pair<token, program_loc> > locs;
    vector<token> already_added;
    for (const auto b : p) {
      if (is_call_block(b)) {
	token ic = *find_if(b.begin(), b.end(), is_register('P'));
	token line_no('N', ic.v);
	if (find(already_added.begin(), already_added.end(), line_no) ==
	    already_added.end()) {
	  program_loc loc = find_loc(p, line_no);
	  locs.push_back(pair<token, program_loc>(line_no, loc));
	  already_added.push_back(line_no);
	}
      }
    }
    return locs;
  }

  bool is_cut(const machine_state& s) {
    return (s.active_move_type != FAST_MOVE) && !(s.x->is_omitted() || s.y->is_omitted() || s.z->is_omitted());
  }

  bool is_move(const machine_state& s) {
    return (s.active_move_type != UNKNOWN_MOVE_TYPE) && !(s.x->is_omitted() && s.y->is_omitted() && s.z->is_omitted());
  }
  
  bool spindle_off(const machine_state& s) {
    return (s.spindle_setting == SPINDLE_OFF ||
	    s.spindle_setting == SPINDLE_STATE_UNKNOWN);
  }

  bool is_analyzable(const vector<machine_state>& toolpath) {
    if (toolpath.size() == 0) { return false; }
    if (!all_of(toolpath.begin(), toolpath.end(),
		  [](const machine_state& s)
		{ return s.active_coord_system == G54_COORD_SYSTEM; })) {
      return false;
    }
    auto ps = program_position_table(toolpath);
    auto ptbl = select_column(G54_COORD_SYSTEM, ps);
    auto p_it = ptbl.begin();
    for (auto t_it = toolpath.begin(); t_it != toolpath.end() - 1; ++t_it, ++p_it) {
      auto s = *(t_it + 1);
      auto last_pos = *(p_it);
      auto next_pos = *(p_it + 1);
      if (is_cut(s) && !(last_pos.is_lit() && next_pos.is_lit())) {
	cout << "Cannot analyze toolpath, not all cut start and end locations are known" << endl;
	return false;
      }
    }
    return true;
  }

  bool is_canned_cycle(const machine_state& s) {
    move_type m = s.active_move_type;
    return m == CANNED_CYCLE_73_MOVE ||
      m == CANNED_CYCLE_81_MOVE ||
      m == CANNED_CYCLE_83_MOVE ||
      m == CANNED_CYCLE_84_MOVE ||
      m == CANNED_CYCLE_85_MOVE;
  }

  bool is_climb_milling(const machine_state& s) {
    if (s.tool_radius_comp == TOOL_RADIUS_COMP_LEFT) {
      return s.spindle_setting == SPINDLE_CLOCKWISE;
    } else if (s.tool_radius_comp == TOOL_RADIUS_COMP_RIGHT) {
      return s.spindle_setting == SPINDLE_COUNTERCLOCKWISE;
    } else {
      assert(false);
    }
  }

  void sanity_check_machine_state(const machine_state& s) {
    if (is_cut(s)) {
      if (spindle_off(s)) {
	cout << "Spindle off during cut: " << endl;
	cout << s << endl;
	assert(false);
      }
    }
  }

  
}
