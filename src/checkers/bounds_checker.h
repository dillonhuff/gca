#ifndef GCA_BOUNDS_CHECKER_H
#define GCA_BOUNDS_CHECKER_H

#include "core/basic_states.h"
#include "core/pass.h"

namespace gca {

  class bounds_checker_state : public per_instr_state {
  protected:
    double x_min, y_min, z_min, x_max, y_max, z_max;

    bool in_bounds(double lb, double x, double rb) const {
      return lb <= x && x <= rb;
    }

    bool check_point(point p) {
      bool res_x = in_bounds(x_min, p.x, x_max);
      bool res_y = in_bounds(y_min, p.y, y_max);
      bool res_z = in_bounds(z_min, p.z, z_max);
      if (!res_x) {
      	add_warning("is not in X bounds");
      }
      if (!res_y) {
      	add_warning("is not in Y bounds");
      }
      if (!res_z) {
      	add_warning("is not in Z bounds");
      }
      return res_x && res_y && res_z;
    }

  public:
  bounds_checker_state(pass* tp,
		       double x_minp,
		       double x_maxp,
		       double y_minp,
		       double y_maxp,
		       double z_minp,
		       double z_maxp) :
    x_min(x_minp), y_min(y_minp), z_min(z_minp),
      x_max(x_maxp),y_max(y_maxp), z_max(z_maxp) {
      t = tp;
    }
    
    virtual void update_G0(move_instr& ist) {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      check_point(ps->after);
    }

    virtual void update_G1(move_instr& ist) {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      check_point(ps->after);
    }
    
  };

  int check_bounds(gprog* p, orientation orient,
		   double x_minp,
		   double x_maxp,
		   double y_minp,
		   double y_maxp,
		   double z_minp,
		   double z_maxp) {
    pass ps;
    position_state pos_s(&ps, point(0, 0, 0));
    bounds_checker_state bound_s(&ps, x_minp, x_maxp, y_minp, y_maxp, z_minp, z_maxp);
    orientation_state orient_s(&ps, orient);
    ps.add_state(GCA_POSITION_STATE, &pos_s);
    ps.add_state(GCA_BOUNDS_CHECKER_STATE, &bound_s);
    ps.add_state(GCA_ORIENTATION_STATE, &orient_s);
    ps.exec(p);
    return ps.num_warns;
  }

}

#endif
