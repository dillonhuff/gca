#include "pass.h"

namespace gca {

  void pass::exec(gprog* prog) {
    p = prog;
    for (int i = 0; i < p->size(); i++) {
      for (state_map::iterator it = states.begin();
	   it != states.end(); ++it) {
	state* s = it->second;
	s->update();
      }
    }
  }
  
  state* state::get_state(state_name n)  {
    assert(t != NULL); return t->get_state(n);
  }
}
