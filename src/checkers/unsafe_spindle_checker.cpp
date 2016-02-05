#include "checkers/unsafe_spindle_checker.h"

namespace gca {
  
  int check_for_unsafe_spindle_on(const vector<int>& no_spindle_tools,
				  int default_tool,
				  gprog* p) {
    spindle_callback c(no_spindle_tools, default_tool);
    int num_warns = 0;
    for (unsigned i = 0; i < p->size(); i++) {
      num_warns += c.call(p, i, (*p)[i]);
    }
    return num_warns;
  }

}
