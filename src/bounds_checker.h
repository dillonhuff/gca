#ifndef GCA_BOUNDS_CHECKER_H
#define GCA_BOUNDS_CHECKER_H

#include "basic_states.h"
#include "pass.h"

#define GCA_BOUNDS_CHECKER_STATE 2000

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
      state* s = get_state(GCA_WARNING_STATE);
      warning_state* ws = static_cast<warning_state*>(s);
      if (!res_x) {
      	ws->add_warning("is not in X bounds");
      }
      if (!res_y) {
      	ws->add_warning("is not in Y bounds");
      }
      if (!res_z) {
      	ws->add_warning("is not in Z bounds");
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
    
    virtual void update_G0(instr* ist) {
      position_state* ps = static_cast<position_state*>(t->get_state(GCA_POSITION_STATE));
      check_point(ps->after);
    }

    virtual void update_G1(instr* ist) {
      position_state* ps = static_cast<position_state*>(t->get_state(GCA_POSITION_STATE));
      check_point(ps->after);
    }
    
  };

  class bounds_checker : public pass {
  protected:
    current_instr_state cis;
    position_state ps;
    warning_state s;
    orientation_state orient_s;
    bounds_checker_state bound_s;

  public:
  bounds_checker(orientation def,
		 double x_minp,
		 double x_maxp,
		 double y_minp,
		 double y_maxp,
		 double z_minp,
		 double z_maxp) :
    cis(this), ps(this, point(0, 0, 0)), orient_s(this, def),
      bound_s(this, x_minp, x_maxp, y_minp, y_maxp, z_minp, z_maxp) {
      states[GCA_INSTR_STATE] = &cis;
      states[GCA_WARNING_STATE] = &s;
      states[GCA_POSITION_STATE] = &ps;
      states[GCA_ORIENTATION_STATE] = &orient_s;
      states[GCA_BOUNDS_CHECKER_STATE] = &bound_s;
    }

  };

}

#endif
