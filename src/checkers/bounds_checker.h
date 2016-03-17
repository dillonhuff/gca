#ifndef GCA_BOUNDS_CHECKER_H
#define GCA_BOUNDS_CHECKER_H

#include "core/basic_states.h"
#include "core/pass.h"

namespace gca {

  int check_bounds(gprog* p, orientation orient,
		   double x_minp,
		   double x_maxp,
		   double y_minp,
		   double y_maxp,
		   double z_minp,
		   double z_maxp);

}

#endif
