#include "geometry/polyline.h"
#include "system/algorithm.h"

namespace gca {

  bool pointwise_within_eps(const polyline& p, const polyline& q, double tol) {
    if (!(p.num_points() == q.num_points())) { return false; }
    return mismatch(p.begin(), p.end(), q.begin(),
		    [tol](point l, point r) { return within_eps(l, r, tol); }).first == p.end();
  }

  template<typename T>
  struct maybe {
    bool just;
    T t;
    maybe() : just(false), t() {}
    maybe(T tp) : just(true), t(tp) {}
  };

  // Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
  // intersect the intersection point may be stored in the floats i_x and i_y.
  char get_line_intersection(double p0_x, double p0_y, double p1_x, double p1_y, 
    double p2_x, double p2_y, double p3_x, double p3_y, double *i_x, double *i_y) {
    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
        if (i_x != NULL)
            *i_x = p0_x + (t * s1_x);
        if (i_y != NULL)
            *i_y = p0_y + (t * s1_y);
        return 1;
    }

    return 0; // No collision
}  
  maybe<point> intersection_point_2d(line p, line l) {
    double x;
    double y;
    double z = p.start.z;
    cout << "p = " << p << endl;
    cout << "l = " << l << endl;
    if (get_line_intersection(p.start.x, p.start.y,
			      p.end.x, p.end.y,
			      l.start.x, l.start.y,
			      l.end.x, l.end.y,
			      &x, &y)) {
      return maybe<point>(point(x, y, z));
    } else {
      return maybe<point>();
    }
  }

  point trim_or_extend(line prev, line next) {
    auto intersection = intersection_point_2d(prev, next);
    if (intersection.just) {
      return intersection.t;
    } else {
      // TODO: Insert the slope computation here
      return point(0, 0, 0);
    }
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
    new_points[0] = initial_lines[0].start;
    apply_between(initial_lines.begin(), initial_lines.end(),
		  new_points.begin() + 1,
		  trim_or_extend);
    new_points.push_back(initial_lines.back().end);
    polyline off(new_points);
    return off;
  }

}
