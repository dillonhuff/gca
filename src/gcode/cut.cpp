#include "gcode/cut.h"
#include "utils/check.h"

namespace gca {

  ostream& operator<<(ostream& stream, const cut& c) {
    c.print(stream);
    stream << c.settings;
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
      DBG_ASSERT(f->is_lit());
      fr = static_cast<lit*>(f)->v;
    } else {
      // This is the fast feedrate for HAAS VF1
      // Q: Does the HAAS actually go that fast during
      // G0 moves?
      fr = 1000;
    }
    return (c->get_end() - c->get_start()).len() / fr;
  }

  double execution_time_seconds(const std::vector<cut*>& cs) {
    return execution_time_minutes(cs)*60.0;
  }

  double cut_execution_time_seconds(const cut* c) {
    return cut_execution_time_minutes(c) * 60;
  }

  double execution_time_minutes(const vector<cut*>& path) {
    double exec_time = 0.0;
    for (auto c : path) { exec_time += cut_execution_time_minutes(c); }
    return exec_time;
  }

  box bound_paths(const vector<vector<cut*>>& paths) {
    vector<box> path_boxes;
    for (auto path : paths) {
      path_boxes.push_back(path_bounds(path));
    }
    return bound_boxes(path_boxes);
  }

  double infer_material_height(const vector<vector<cut*>>& paths, double offset) {
    double material_height = -1000000;
    for (auto p : paths) {
      for (auto c : p) {
	if (!c->is_safe_move() && !is_vertical(c)) {
	  double z = max(c->get_start().z, c->get_end().z);
	  if (z > material_height) {
	    material_height = z;
	  }
	}
      }
    }
    return material_height + offset;
  }

  double infer_safe_height(const vector<vector<cut*>>& paths) {
    // TODO: Proper floating point max
    double safe_height = 10000000;
    for (auto p : paths) {
      for (auto c : p) {
	if (c->is_safe_move()) {
	  double z = max(c->get_start().z, c->get_end().z);
	  if (z < safe_height) {
	    safe_height = z;
	  }
	}
      }
    }
    return safe_height - 0.1;
  }

  int get_active_tool_no(const vector<cut*>& path) {
    auto c = *find_if(path.begin(), path.end(),
		      [](const cut* c) { return !c->is_safe_move(); });
    auto tn = c->settings.active_tool; //path.front()->settings.active_tool;
    if (!(tn->is_ilit())) {
      cout << "ERROR" << endl;
      cout << *c << endl;
      cout << "Active tool = " << *(c->settings.active_tool) << endl;
      DBG_ASSERT(false);
    }
    auto tl = static_cast<ilit*>(tn);
    int current_tool_no = tl->v;
    return current_tool_no;
  }

  double extract_spindle_speed(const cut* c) {
    auto tn = c->settings.spindle_speed;
    if (tn->is_omitted()) {
      cout << "ERROR in get_spindle_speed" << endl;
      cout << "is lit ? " << tn->is_lit() << endl;
      cout << "is omitted ? " << tn->is_omitted() << endl;
      cout << *c << endl;
      DBG_ASSERT(false);
    } else if (tn->is_ilit()) {
      auto ss = static_cast<ilit*>(tn);
      return ss->v;
    } else if (tn->is_lit()) {
      auto ss = static_cast<lit*>(tn);
      return ss->v;
    } else {
      DBG_ASSERT(false);
    }
  }

  double get_spindle_speed(const vector<cut*>& path) {
    auto c = *find_if(path.begin(), path.end(),
		      [](const cut* c) { return !c->is_safe_move(); });
    return extract_spindle_speed(c);
  }

  double get_spindle_speed(const cut* c) {
    return extract_spindle_speed(c);
  }
  
}
