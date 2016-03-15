#include <stack>

#include "analysis/unfold.h"

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

  vector<block> unfold_gprog(const vector<block>& p) {
    vector<block> bs;
    vector<pair<token, program_loc> > subroutine_starts = compute_starts(p);
    stack<vector<block>::const_iterator> istack;
    vector<block>::const_iterator it = p.begin();
    while (it < p.end()) {
      const block& b = *it;
      if (is_call_block(b)) {
	++it;
	istack.push(it);
	it = find_called_subroutine(b, subroutine_starts);
      } else if (is_ret_block(*it)) {
	it = istack.top();
	istack.pop();
      } else if (is_end_block(*it)) {
	bs.push_back(*it);
	break;
      } else {
	bs.push_back(*it);
	++it;
      }
    }
    return bs;
  }

}
