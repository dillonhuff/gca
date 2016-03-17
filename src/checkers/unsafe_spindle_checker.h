#ifndef GCA_UNSAFE_SPINDLE_CHECKER_H
#define GCA_UNSAFE_SPINDLE_CHECKER_H

#include <algorithm>

#include "core/callback.h"
#include "core/gprog.h"

namespace gca {

  int check_for_unsafe_spindle_on(const vector<int>& no_spindle_tools,
				  int default_tool,
				  gprog* p);

}

#endif
