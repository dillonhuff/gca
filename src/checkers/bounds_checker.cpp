#include "checkers/bounds_checker.h"

namespace gca {

  int check_bounds(gprog* p, orientation orient,
		   double x_minp,
		   double x_maxp,
		   double y_minp,
		   double y_maxp,
		   double z_minp,
		   double z_maxp) {
    pass ps;
    position_state pos_s(ps, point(0, 0, 0));
    bounds_checker_state bound_s(ps, x_minp, x_maxp, y_minp, y_maxp, z_minp, z_maxp);
    orientation_state orient_s(ps, orient);
    ps.add_state(GCA_POSITION_STATE, &pos_s);
    ps.add_state(GCA_BOUNDS_CHECKER_STATE, &bound_s);
    ps.add_state(GCA_ORIENTATION_STATE, &orient_s);
    ps.exec(p);
    return ps.num_warns;
  }

}
