#include "pass.h"
namespace gca {

  void pass::exec(gprog* prog) {
    cout << "Start exec" << endl;
    p = prog;
    for (int i = 0; i < p->size(); i++) {
      cout << "i = " << i << endl;
      for (state_map::iterator it = states.begin();
	   it != states.end(); ++it) {
	cout << "Updating state" << endl;
	state* s = it->second;
	s->update();
      }
    }
    cout << "End exec" << endl;
  }
  
  state* state::get_state(state_name n)  {
    assert(t != NULL); return t->get_state(n);
  }
}
