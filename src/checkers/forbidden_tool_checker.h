#ifndef GCA_FORBIDDEN_TOOL_CHECKER_H
#define GCA_FORBIDDEN_TOOL_CHECKER_H

#include <algorithm>

#include "core/callback.h"
#include "core/gprog.h"

namespace gca {

  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools, gprog* p);

}

#endif
