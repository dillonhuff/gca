#ifndef GCA_UNSAFE_SPINDLE_CHECKER_H
#define GCA_UNSAFE_SPINDLE_CHECKER_H

#include <algorithm>

#include "analysis/utils.h"
#include "core/gprog.h"

namespace gca {

  int check_for_unsafe_spindle_on(const vector<int>& no_spindle_tools,
				  int default_tool,
				  gprog* p);

  int check_for_unsafe_spindle_on(const vector<int>& no_spindle_tools,
				  int default_tool,
				  const vector<block>& ws);
  
}

#endif
