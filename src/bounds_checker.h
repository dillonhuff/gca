#ifndef GCA_BOUNDS_CHECKER_H
#define GCA_BOUNDS_CHECKER_H

#include "per_instr_checker.h"

namespace gca {

  class bounds_checker : public per_instr_checker {
  protected:
    double x_min, y_min, z_min, x_max, y_max, z_max;

    bool in_bounds(double lb, double x, double rb) const {
      return lb <= x && x <= rb;
    }
    
    virtual bool check_instr(ostream& out, instr* p) const {
      if (p->is_G0() || p->is_G1()) { //p->c == GCA_G && (p->v == 0 || p->v == 1)) {
	bool res_x = in_bounds(x_min, p->pos().x, x_max);
	bool res_y = in_bounds(y_min, p->pos().y, y_max);
	bool res_z = in_bounds(z_min, p->pos().z, z_max);
	if (!res_x) {
	  out << "Warning: " << *p << " is not in X bounds ";
	  out << "(" << x_min << ", " << x_max << ")" << endl;
	}
	if (!res_y) {
	  out << "Warning: " << *p << " is not in Y bounds ";
	  out << "(" << y_min << ", " << y_max << ")" << endl;
	}
	if (!res_z) {
	  out << "Warning: " << *p << " is not in Z bounds ";
	  out << "(" << z_min << ", " << z_max << ")" << endl;
	}
	return res_x && res_y && res_z;
      }
      return true;
    }
    
  public:
    bounds_checker(double x_minp,
		   double x_maxp,
		   double y_minp,
		   double y_maxp,
		   double z_minp,
		   double z_maxp) :
    x_min(x_minp), y_min(y_minp), z_min(z_minp),
      x_max(x_maxp),y_max(y_maxp), z_max(z_maxp) {

    }

  };

}

#endif
