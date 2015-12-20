#ifndef GCA_BOUNDS_CHECKER_H
#define GCA_BOUNDS_CHECKER_H

#include "instr_bounds_checker.h"
#include "per_instr_checker.h"

namespace gca {

  class bounds_checker : public per_instr_checker {
  protected:
    instr_bounds_checker b;
    
  public:
    bounds_checker(double x_min,
		   double x_max,
		   double y_min,
		   double y_max,
		   double z_min,
		   double z_max) :
    b(instr_bounds_checker(x_min, x_max, y_min, y_max, z_min, z_max)) {
      c = &b;
    }

  };

}

#endif
