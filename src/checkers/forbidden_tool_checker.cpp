#include "checkers/forbidden_tool_checker.h"

namespace gca {

  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools,
				       gprog* p) {
    toolchange_callback c(permitted_tools);
    int num_warns = 0;
    for (unsigned i = 0; i < p->size(); i++) {
      num_warns += c.call(p, i, (*p)[i]);
    }
    return num_warns;
  }

}
