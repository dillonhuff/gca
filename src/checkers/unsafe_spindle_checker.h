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
    bool spindle_on;
    
  spindle_callback(const vector<int>& pt, int st) :
    no_spindle_tools(pt), active_tool(st),
      spindle_speed(0), spindle_on(false) {}
    
    int call_T(gprog* p, int i, t_instr* is) {
      active_tool = is->num;
      return call_default(p, i, is);
    }

    int call_S(gprog* p, int i, s_instr* is) {
      spindle_speed = is->num;
      return call_default(p, i, is);
    }

    int call_M3(gprog* p, int i, m3_instr* is) {
      spindle_on = true;
      return call_default(p, i, is);
    }

    int call_M5(gprog* p, int i, m5_instr* is) {
      spindle_on = false;
      return call_default(p, i, is);
    }
    
    int call_default(gprog* p, int i, instr* is) {
      if (spindle_on && spindle_speed > 0 &&
	  find(no_spindle_tools.begin(),
	       no_spindle_tools.end(),
	       active_tool) != no_spindle_tools.end()) {
	cout << "Warning at position: " << *is << " spindle is on with active tool T" << active_tool << endl;
	return 1;
      }
      return 0;
    }
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
