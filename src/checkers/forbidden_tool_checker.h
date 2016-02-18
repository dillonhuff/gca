#ifndef GCA_FORBIDDEN_TOOL_CHECKER_H
#define GCA_FORBIDDEN_TOOL_CHECKER_H

#include <algorithm>

#include "core/callback.h"
#include "core/gprog.h"

namespace gca {

  class toolchange_callback : public per_instr_callback<int> {
  public:
    const vector<int>& permitted_tools;
    
    toolchange_callback(const vector<int>& pt) : permitted_tools(pt) {}
    
    int call_T(gprog* p, int i, t_instr* is) {
      if (find(permitted_tools.begin(),
	       permitted_tools.end(),
	       is->num) == permitted_tools.end()) {
	cout << "Warning at position: " << *is << " is a forbidden toolchange" << endl;
	return 1;
      }
      return 0;
    }

    int call_default(gprog* p, int i, instr* is) { return 0; }
  };

  int check_for_forbidden_tool_changes(const vector<int>& permitted_tools, gprog* p);

}

#endif
