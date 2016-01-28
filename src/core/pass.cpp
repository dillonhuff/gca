#include "pass.h"

namespace gca {

  void pass::update(instr* ist) {
    for (state_map::iterator it = states.begin();
	 it != states.end(); ++it) {
      state* s = it->second;
      s->update(*ist);
    }    
  }

  void pass::exec(gprog* prog) {
    p = prog;
    for (i = 0; i < p->size(); i++) {
      update((*p)[i]);
    }
  }
  
}
