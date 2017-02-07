#include "transformers/feed_changer.h"

namespace gca {
  
  vector<block> change_feeds(const vector<block>& p,
			     value* initial_feedrate,
			     value* new_feedrate) {
    vector<block> np;
    for (auto bp : p) {
      block b;
      for (auto t : bp) {
	if (t.tp() == ICODE && t.c == 'F' && t.get_value() == *initial_feedrate)
	  { t.set_value_ptr(new_feedrate); } //t.v = new_feedrate; }
	b.push_back(t);
      }
      np.push_back(b);
    }
    return np;
  }

}
