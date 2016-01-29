#ifndef GCA_UNSAFE_SPINDLE_CHECKER_H
#define GCA_UNSAFE_SPINDLE_CHECKER_H

#include "core/callback.h"
#include "core/gprog.h"

namespace gca {

  class spindle_callback : public per_instr_callback<int> {
  public:
    const vector<int>& no_spindle_tools;
    int active_tool;
    int spindle_speed;
    
  spindle_callback(const vector<int>& pt, int st) :
    no_spindle_tools(pt), active_tool(st) {}
    
    int call_T(gprog* p, int i, t_instr* is) {
      active_tool = is->num;
    }

    int call_default(gprog* p, int i, instr* is) { return 0; }
  };

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

#endif
