#include "gcode/lexer.h"
#include "checkers/forbidden_tool_checker.h"

namespace gca {

  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools,
				       const vector<block>& ws) {
    int num_warns = 0;
    for (auto b : ws) {
      for (auto w : b) {
	if (w.tp() == ICODE &&
	    w.c == 'T') {
	  int i = static_cast<ilit*>(w.get_value_ptr())->v;
	  if (find(permitted_tools.begin(), permitted_tools.end(), i) == permitted_tools.end())
	    { num_warns++; }
	}
      }
    }
    return num_warns;
  }

}
