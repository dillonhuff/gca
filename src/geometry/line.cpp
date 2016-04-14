#include <cassert>

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

  maybe<double> slope_2d(line l) {
    if (within_eps(l.end.x, l.start.x, 0.000001)) {
      return maybe<double>();
    } else {
      return maybe<double>((l.end.y - l.start.y) / (l.end.x - l.start.x));
    }
    
  }

  maybe<double> y_intersect_2d(line l) {
    maybe<double> sm = slope_2d(l);
    if (sm.just) {
      return maybe<double>(l.start.y - (sm.t)*l.start.x);
    } else {
      return maybe<double>();
    }
  }

  maybe<point> line_intersection_2d(line prev, line next) {
    double z = prev.start.z;
    maybe<double> p_slope_m = slope_2d(prev);
    maybe<double> n_slope_m = slope_2d(next);
    if (p_slope_m.just && n_slope_m.just) {
      auto p_slope = p_slope_m.t;
      auto n_slope = n_slope_m.t;
      maybe<double> p_b_m = y_intersect_2d(prev);
      maybe<double> n_b_m = y_intersect_2d(next);
      if (p_b_m.just && n_b_m.just) {
	double p_b = p_b_m.t;
	double n_b = n_b_m.t;
	if (within_eps(p_slope, n_slope) && !within_eps(p_b, n_b)) {
	  return maybe<point>();
	}
	double x = (p_b - n_b) / (n_slope - p_slope);
	double y = p_slope*x + p_b;
	return maybe<point>(point(x, y, z));
      } else {
	assert(false);
      }
    } else if (p_slope_m.just) {
      double x = next.start.x;
      return maybe<point>(point(x, p_slope_m.t*x + y_intersect_2d(prev).t, z));
    } else if (n_slope_m.just) {
      double x = prev.start.x;
      return maybe<point>(point(x, n_slope_m.t*x + y_intersect_2d(next).t, z));
    } else {
      assert(false);
    }
  }

  maybe<point> trim_or_extend(line prev, line next) {
    auto intersection = segment_intersection_2d(prev, next);
    if (intersection.just) {
      return intersection.t;
    } else {
      return line_intersection_2d(prev, next);
    }
  }

  point trim_or_extend_unsafe(line prev, line next) {
    maybe<point> p = trim_or_extend(prev, next);
    assert(p.just);
    return p.t;
  }

}
