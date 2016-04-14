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
  maybe<point> segment_intersection_2d(line p, line l) {
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

  double slope_2d(line l) {
    return (l.end.y - l.start.y) / (l.end.x - l.start.x);
  }

  double y_intersect_2d(line l) {
    return l.start.y - slope_2d(l)*l.start.x;
  }

  point line_intersection_2d(line prev, line next) {
    double z = prev.start.z;
    double p_slope = slope_2d(prev);
    double n_slope = slope_2d(next);
    double p_b = y_intersect_2d(prev);
    double n_b = y_intersect_2d(next);
    double x = (p_b - n_b) / (n_slope - p_slope);
    double y = p_slope*x + p_b;
    return point(x, y, z);
  }

  point trim_or_extend(line prev, line next) {
    auto intersection = segment_intersection_2d(prev, next);
    if (intersection.just) {
      return intersection.t;
    } else {
      return line_intersection_2d(prev, next);
    }
  }

}
