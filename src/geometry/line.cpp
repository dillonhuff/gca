#include "geometry/line.h"

namespace gca {
  
  ostream& operator<<(ostream& out, line l) {
    cout << l.start << " -> " << l.end << endl;
    return out;
  }

  bool same_line(const line l, const line r) {
    bool ss = within_eps(l.start, r.start);
    bool ee = within_eps(l.end, r.end);
    bool se = within_eps(l.start, r.end);
    bool es = within_eps(l.end, r.start);
    return (ss && ee) || (se && es);
  }

  int count_in(const line l, const vector<line> ls) {
    return count_if(ls.begin(), ls.end(), [l](const line r)
		    { return same_line(l, r); });
  }

  bool adj_segment(const line l, const line r) {
    return within_eps(l.start, r.end) ||
      within_eps(l.end, r.start);
  }

}
