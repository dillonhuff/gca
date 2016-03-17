#include <sstream>

#include "analysis/machine_state.h"
#include "analysis/utils.h"
#include "checkers/unsafe_spindle_checker.h"

namespace gca {
  
  int check_for_unsafe_spindle_on(const vector<int>& no_spindle_tools,
				  int default_tool,
				  gprog* p) {
    stringstream s;
    s << *p;
    auto ws = lex_gprog(s.str());
    auto ms = all_program_states(ws);
    int num_warns = 0;
    for (auto s : ms) {
      if (find(no_spindle_tools.begin(),
	       no_spindle_tools.end(),
	       static_cast<ilit*>(s.last_referenced_tool)->v) != no_spindle_tools.end() &&
	  !spindle_off(s)) {
	num_warns++;
      }
    }

    return num_warns;
  }

}
