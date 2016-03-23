#ifndef GCA_FORBIDDEN_TOOL_CHECKER_H
#define GCA_FORBIDDEN_TOOL_CHECKER_H

#include "analysis/machine_state.h"
#include "core/gprog.h"

namespace gca {

  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools, gprog* p);

  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools,
				       const vector<block>& ws);
}

#endif
