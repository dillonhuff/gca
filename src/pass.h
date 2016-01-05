#ifndef GCA_PASS_H
#define GCA_PASS_H

#include <map>

#include "gprog.h"
#include "state.h"

using namespace std;

namespace gca {

  typedef int state_name;
  typedef map<state_name, state*> state_map;

  class pass {
  protected:
    
    state_map states;
    
  public:
    state* get_state(state_name n) { return states[n]; }
    gprog* p;
    virtual void exec(gprog* prog) {
      p = prog;
      for (int i = 0; i < p->size(); i++) {
	for (state_map::iterator it = states.begin();
	     it != states.end(); ++it) {
	  state* s = it->second;
	  s->update();
	}
      }
    }
  };
}

#endif
