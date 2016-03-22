#include <iostream>

#include "geometry/box.h"

using namespace std;

namespace gca {

  ostream& operator<<(ostream& out, const box& b) {
    out << "X min = " << b.x_min << endl;
    out << "X max = " << b.x_max << endl;
    out << "Y min = " << b.y_min << endl;
    out << "Y max = " << b.y_max << endl;
    return out;
  }

  template<typename T>
  bool intervals_overlap(pair<T, T> i, pair<T, T> j) {
    return i.second >= j.first || j.second >= i.first;
  }
  
  bool overlap(const box l, const box r) {
    bool x_overlap = intervals_overlap(pair<double, double>(l.x_min, l.x_max),
				       pair<double, double>(r.x_min, r.x_max));
    bool y_overlap = intervals_overlap(pair<double, double>(l.y_min, l.y_max),
				       pair<double, double>(r.y_min, r.y_max));
    return x_overlap && y_overlap;
  }

}
