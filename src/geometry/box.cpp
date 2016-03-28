#include <iostream>

#include "geometry/box.h"

using namespace std;

namespace gca {

  ostream& operator<<(ostream& out, const box& b) {
    out << "X min = " << b.x_min << endl;
    out << "X max = " << b.x_max << endl;
    out << "Y min = " << b.y_min << endl;
    out << "Y max = " << b.y_max << endl;
    out << "Y min = " << b.z_min << endl;
    out << "Y max = " << b.z_max << endl;
    return out;
  }

  // TODO: Add some unit tests
  template<typename T>
  bool intervals_overlap(pair<T, T> i, pair<T, T> j) {
    return (j.first <= i.second && i.second <= j.second) ||
      (j.first <= i.first && i.first <= j.second) ||
      (i.first <= j.first && i.first <= j.second &&
       j.first <= i.second && j.second <= i.second);
  }
  
  bool overlap(const box l, const box r) {
    bool x_overlap = intervals_overlap(pair<double, double>(l.x_min, l.x_max),
				       pair<double, double>(r.x_min, r.x_max));
    bool y_overlap = intervals_overlap(pair<double, double>(l.y_min, l.y_max),
				       pair<double, double>(r.y_min, r.y_max));
    bool z_overlap = intervals_overlap(pair<double, double>(l.z_min, l.z_max),
				       pair<double, double>(r.z_min, r.z_max));
    return x_overlap && y_overlap && z_overlap;
  }

  box bound_positions(const vector<point> pts) {
    assert(pts.size() > 0);
    auto xminmax = minmax_element(pts.begin(), pts.end(),
				  [](const point l, const point r)
				  { return l.x < r.x; });
    assert(xminmax.first != pts.end());
    auto yminmax = minmax_element(pts.begin(), pts.end(),
				  [](const point l, const point r)
				  { return l.y < r.y; });
    auto zminmax = minmax_element(pts.begin(), pts.end(),
				  [](const point l, const point r)
				  { return l.z < r.z; });
    assert(yminmax.first != pts.end());
    return box((*(xminmax.first)).x, (*(xminmax.second)).x,
	       (*(yminmax.first)).y, (*(yminmax.second)).y,
	       (*(zminmax.first)).z, (*(zminmax.second)).z);
  }

}
