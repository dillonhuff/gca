#ifndef GCA_INSTR_BOUNDS_CHECKER_H
#define GCA_INSTR_BOUNDS_CHECKER_H

#include "instr_checker.h"

namespace gca {

  class instr_bounds_checker : public instr_checker {
  public:

    double x_min, y_min, z_min, x_max, y_max, z_max;
    
    instr_bounds_checker(double x_minp,
			 double x_maxp,
			 double y_minp,
			 double y_maxp,
			 double z_minp,
			 double z_maxp) :
    x_min(x_minp), y_min(y_minp), z_min(z_minp),
      x_max(x_maxp),y_max(y_maxp), z_max(z_maxp) {
    }

    bool in_bounds(double lb, double x, double rb) {
      return lb <= x && x <= rb;
    }
    
    virtual bool check(ostream& out, instr* p) {
      if (p->c == GCA_G && (p->v == 0 || p->v == 1)) {
	bool res_x = in_bounds(x_min, p->x, x_max);
	bool res_y = in_bounds(y_min, p->y, y_max);
	bool res_z = in_bounds(z_min, p->z, z_max);
	if (!res_x) {
	  out << "Warning: " << *p << " is not in X bounds ";
	  out << "(" << x_min << ", " << x_max << ")" << endl;
	}
	if (!res_y) {
	  out << "Warning: " << *p << " is not in Y bounds";
	  out << "(" << y_min << ", " << y_max << ")" << endl;
	}
	if (!res_z) {
	  out << "Warning: " << *p << " is not in Z bounds";
	  out << "(" << z_min << ", " << z_max << ")" << endl;	  
	}
	return res_x && res_y && res_z;
      }
      return true;
    }
    
  };
  
}

#endif
