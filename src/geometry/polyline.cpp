#include "geometry/line.h"
#include "geometry/polyline.h"
#include "system/algorithm.h"

namespace gca {

  bool pointwise_within_eps(const polyline& p, const polyline& q, double tol) {
    if (!(p.num_points() == q.num_points())) { return false; }
    return mismatch(p.begin(), p.end(), q.begin(),
		    [tol](point l, point r) { return within_eps(l, r, tol); }).first == p.end();
  }

  // TODO: Add assertion to check for simplicity, e.g. the only
  // intersection of the lines is potentially the first and last point
  polyline offset(const polyline& p, double degrees, double n) {
    assert(p.num_points() > 1);
    vector<line> initial_lines;
    for (auto l : p.lines()) {
      auto offset_vec = n * ((l.end - l.start).normalize()).rotate_z(degrees);
      initial_lines.push_back(line(l.start + offset_vec, l.end + offset_vec));
    }
    vector<point> new_points(initial_lines.size());
    apply_between(initial_lines.begin(), initial_lines.end(),
		  new_points.begin() + 1,
		  trim_or_extend_unsafe);
    // TODO: Allow this tolerance to be set by the api
    if (!within_eps(p.front(), p.back(), 0.000001)) {
      new_points[0] = initial_lines[0].start;
      new_points.push_back(initial_lines.back().end);
    } else {
      assert(p.num_points() > 3);
      point n = trim_or_extend_unsafe(initial_lines[initial_lines.size() - 1],
				      initial_lines[0]);
      new_points[0] = n;
      new_points.push_back(n);
    }
    polyline off(new_points);
    return off;
  }

}
