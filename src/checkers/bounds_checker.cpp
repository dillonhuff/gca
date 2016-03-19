#include <sstream>

#include "analysis/position_table.h"
#include "checkers/bounds_checker.h"

namespace gca {

  struct bounds {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
    bounds(double x_minp,
	   double x_maxp,
	   double y_minp,
	   double y_maxp,
	   double z_minp,
	   double z_maxp) :
      x_min(x_minp),
      x_max(x_maxp),
      y_min(y_minp),
      y_max(y_maxp),
      z_min(z_minp),
      z_max(z_maxp) {}

  };

  bool in_bounds(double lb, double x, double rb) {
    return lb <= x && x <= rb;
  }
  
  bool entry_in_bounds(const bounds& b, const position_entry& e) {
    position p = e.second;
    if (p.x->is_omitted() && p.y->is_omitted() && p.z->is_omitted()) {
      return true;
    } else if (p.x->is_lit() && p.y->is_lit() && p.z->is_lit()) {
      double x = static_cast<lit*>(p.x)->v;
      double y = static_cast<lit*>(p.y)->v;
      double z = static_cast<lit*>(p.z)->v;
      bool res_x = in_bounds(b.x_min, x, b.x_max);
      bool res_y = in_bounds(b.y_min, y, b.y_max);
      bool res_z = in_bounds(b.z_min, z, b.z_max);
      if (!res_x) {
      	cout << "Warning: " << x << " is not in X bounds" << endl;
      }
      if (!res_y) {
      	cout << "Warning: " << y << " is not in Y bounds" << endl;
      }
      if (!res_z) {
      	cout << "Warning: " << z << " is not in Z bounds" << endl;
      }
      return res_x && res_y && res_z;
    } else {
      return false;
    }
  }

  bool all_entries_in_bounds(const bounds& b, const position_table_row& r) {
    return all_of(r.begin(), r.end(), [b](const position_entry& e)
		  { return entry_in_bounds(b, e); });
  }

  int check_bounds(const vector<block>& ws,
		   orientation orient,
		   double x_minp,
		   double x_maxp,
		   double y_minp,
		   double y_maxp,
		   double z_minp,
		   double z_maxp) {
    machine_state init;
    init.active_distance_mode = orient == GCA_ABSOLUTE ? ABSOLUTE_DISTANCE_MODE : RELATIVE_DISTANCE_MODE;
    auto states = all_program_states(init, ws);
    auto position_table = program_position_table(states);
    bounds b(x_minp, x_maxp, y_minp, y_maxp, z_minp, z_maxp);
    return count_if(position_table.begin(),
		    position_table.end(),
		    [b](const position_table_row& r)
		      { return !all_entries_in_bounds(b, r); });
  }
  
  int check_bounds(gprog* p, orientation orient,
		   double x_minp,
		   double x_maxp,
		   double y_minp,
		   double y_maxp,
		   double z_minp,
		   double z_maxp) {
    stringstream s;
    s << *p;
    auto ws = lex_gprog(s.str());
    return check_bounds(ws,
			orient,
			x_minp,
			x_maxp,
			y_minp,
			y_maxp,
			z_minp,
			z_maxp);
  }
}
