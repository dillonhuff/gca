#include <stack>

#include "analysis/unfold.h"

namespace gca {

  bool is_end_code(const token* t) {
    if (t->tp() == ICODE) {
      const icode* ci = static_cast<const icode*>(t);
      return ci->c == 'M' && (ci->v == ilit(2) || ci->v == ilit(30));
    }
    return false;
  }

  bool is_end_block(const block& t) {
    int num_end_codes = count_if(t.begin(), t.end(), is_end_code);
    assert(num_end_codes == 0 || num_end_codes == 1);
    bool contains_end_instr = num_end_codes == 1;
    return contains_end_instr;
  }

  bool is_ret_code(const token* t) {
    if (t->tp() == ICODE) {
      const icode* ci = static_cast<const icode*>(t);
      return ci->c == 'M' && (ci->v == ilit(99));
    }
    return false;
  }

  bool is_ret_block(const block& t) {
    int num_end_codes = count_if(t.begin(), t.end(), is_ret_code);
    assert(num_end_codes == 0 || num_end_codes == 1);
    bool contains_end_instr = num_end_codes == 1;
    return contains_end_instr;
  }

  bool is_call_code(const token* t) {
    if (t->tp() == ICODE) {
      const icode* ci = static_cast<const icode*>(t);
      return ci->c == 'M' && (ci->v == ilit(97));
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
    token& ic;
    block_contains(token& icp) : ic(icp) {}
    bool operator()(const block& b) {
      return find_if(b.begin(), b.end(), cmp_token_to(&ic)) != b.end();
    }
  };

  struct is_register {
    char c;
    is_register(char cp) : c(cp) {}
    bool operator()(const token* t) const {
      if (t->tp() == ICODE) {
	const icode* ci = static_cast<const icode*>(t);
	return ci->c == c;
      }
      return false;
    }
  };
  
  vector<block> unfold_gprog(const vector<block>& p) {
    vector<block> bs;
    stack<vector<block>::const_iterator> istack;
    vector<block>::const_iterator it = p.begin();
    while (it < p.end()) {
      const block& b = *it;
      if (is_call_block(b)) {
	++it;
	istack.push(it);
	icode* ic = static_cast<icode*>(*find_if(b.begin(), b.end(), is_register('P')));
	icode line_no('N', ic->v);
	it = find_if(p.begin(), p.end(), block_contains(line_no));
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
