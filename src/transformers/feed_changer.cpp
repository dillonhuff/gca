#include "transformers/feed_changer.h"

namespace gca {
  
  vector<block> change_feeds(const vector<block>& p,
			     value* initial_feedrate,
			     value* new_feedrate) {
    vector<block> np;
    for (vector<block>::const_iterator it = p.begin(); it != p.end(); ++it) {
      block b;
      block bp = *it;
      for (block::iterator jt = bp.begin(); jt != bp.end(); ++jt) {
	token t = *jt;
	if (t.tp() == ICODE && t.c == 'F' && *(t.v) == *initial_feedrate) {
	  t.v = new_feedrate;
	}
	b.push_back(t);
      }
      np.push_back(b);
    }
    return np;
  }

}
