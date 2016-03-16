#ifndef GCA_ANALYSIS_UTILS_H
#define GCA_ANALYSIS_UTILS_H

#include "core/lexer.h"
#include "analysis/machine_state.h"

namespace gca {

  bool is_end_code(const token t);
  bool is_end_block(const block& t);
  bool is_ret_code(const token t);
  bool is_ret_block(const block& t);
  bool is_call_code(const token t);  
  bool is_call_block(const block& t);

  struct block_contains {
    const token& ic;
    block_contains(const token& icp) : ic(icp) {}
    bool operator()(const block& b) {
      return find(b.begin(), b.end(), ic) != b.end();
    }
  };

  struct is_register {
    char c;
    is_register(char cp) : c(cp) {}
    bool operator()(const token t) const {
      if (t.tp() == ICODE) {
	return t.c == c;
      }
      return false;
    }
  };

  typedef vector<block>::const_iterator program_loc;
  
  vector<block>::const_iterator
  find_called_subroutine(const block& b,
			 const vector<pair<token, program_loc> >& p);

  program_loc find_loc(const vector<block>& p, const token& t);

  vector<pair<token, program_loc> >
  compute_starts(const vector<block>& p);

  bool is_cut(const machine_state& s);
  bool is_move(const machine_state& s);
  bool spindle_off(const machine_state& s);
  
}

#endif
