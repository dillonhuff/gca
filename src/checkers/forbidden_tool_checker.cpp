#include <sstream>

#include "core/lexer.h"
#include "checkers/forbidden_tool_checker.h"

namespace gca {

  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools,
				       const vector<block>& ws) {
    int num_warns = 0;
    for (auto b : ws) {
      for (auto w : b) {
	if (w.tp() == ICODE &&
	    w.c == 'T') {
	  int i = static_cast<ilit*>(w.v)->v;
	  if (find(permitted_tools.begin(), permitted_tools.end(), i) == permitted_tools.end())
	    { num_warns++; }
	}
      }
    }
    return num_warns;
  }

  
  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools,
				       gprog* p) {
    stringstream s;
    s << *p;
    auto ws = lex_gprog(s.str());
    return check_for_forbidden_tool_changes(permitted_tools, ws);
  }

}
