#ifndef GCA_BOUNDS_CHECKER_H
#define GCA_BOUNDS_CHECKER_H

#include "analysis/machine_state.h"
#include "core/instrs/instr.h"

namespace gca {

  int check_bounds(const vector<block>& ws,
		   orientation orient,
		   double x_minp,
		   double x_maxp,
		   double y_minp,
		   double y_maxp,
		   double z_minp,
		   double z_maxp);
}

#endif
