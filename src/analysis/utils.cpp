#include "analysis/utils.h"

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
			 const vector<pair<token, program_loc> >& p) {
    token ic = *find_if(b.begin(), b.end(), is_register('P'));
    token line_no('N', ic.v);
    for (vector<pair<token, program_loc> >::const_iterator it = p.begin();
	 it != p.end(); ++it) {
      pair<token, program_loc> pr = *it;
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
    for (program_loc it = p.begin(); it != p.end(); ++it) {
      block b = *it;
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

}
