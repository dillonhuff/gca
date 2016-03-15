#include <stack>

#include "analysis/unfold.h"
#include "analysis/utils.h"

namespace gca {

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
