#include "synthesis/cut.h"

namespace gca {

  ostream& operator<<(ostream& stream, const cut& c) {
    c.print(stream);
    return stream;
  }

  ostream& operator<<(ostream& stream, const vector<cut*>& c) {
    for (vector<cut*>::const_iterator it = c.begin(); it != c.end(); ++it) {
      stream << **it << endl;
    }
    return stream;
  }

  bool cmp_cuts(const cut* l, const cut* r) {
    bool res = (*l) == (*r);
    if (!res) {
      cout << "Not equal: " << *l << endl;
      cout << "         : " << *r << endl;
    }
    return res;
  }

  bool same_cut_properties(const cut& l, const cut& r) {
    return (*(l.spindle_speed) == *(r.spindle_speed)) &&
      (*(l.feedrate) == *(r.feedrate)) &&
      (l.tool_no == r.tool_no);
  }

}
