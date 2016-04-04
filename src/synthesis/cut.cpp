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
    return l.settings == r.settings;
  }

  box path_bounds(const vector<cut*>& path) {
    vector<point> bound_pts;
    for (auto c : path) {
      bound_pts.push_back(c->get_start());
      bound_pts.push_back(c->get_end());
      // TODO: Improve sampling
      const int num_points = 10;
      for (int i = 0; i < num_points; i++) {
	double t = static_cast<double>(i) / static_cast<const double>(num_points);
	bound_pts.push_back(c->value_at(t));
      }
    }
    return bound_positions(bound_pts);
  }

  // TODO: Better names for these functions.
  // TODO: Make plane containment testing a property of cuts.
  // It is possible that a cut could have both its start and
  // end in the XY plane even though it moves above that plane
  bool is_vertical(const cut* c) {
    return within_eps(c->get_end().x, c->get_start().x) &&
      within_eps(c->get_end().y, c->get_start().y);
  }

  bool is_horizontal(const cut* c) {
    return within_eps(c->get_end().z, c->get_start().z);
  }

  bool is_prismatic(vector<cut*>& path) {
    return all_of(path.begin(), path.end(),
		  [](const cut* c)
		  { return !c->is_circular_helix_cut() &&
		      (is_vertical(c) || is_horizontal(c)); });
  }

  // TODO: Make this account for cut shape
  double cut_execution_time_minutes(const cut* c) {
    value* f = c->get_feedrate();
    double fr;
    if (!c->is_safe_move()) {
      assert(f->is_lit());
      fr = static_cast<lit*>(f)->v;
    } else {
      // This is the fast feedrate for HAAS VF1
      // Q: Does the HAAS actually go that fast during
      // G0 moves?
      fr = 1000;
    }
    return (c->get_end() - c->get_start()).len() / fr;
  }

  double cut_execution_time_seconds(const cut* c) {
    return cut_execution_time_minutes(c) * 60;
  }

  double execution_time_minutes(const vector<cut*>& path) {
    double exec_time = 0.0;
    for (auto c : path) { exec_time += cut_execution_time_minutes(c); }
    return exec_time;
  }
}
