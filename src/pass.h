#ifndef GCA_PASS_H
#define GCA_PASS_H

#include <map>

#include "gprog.h"

using namespace std;

namespace gca {

  class state;
  
  typedef int state_name;
  typedef map<state_name, state*> state_map;

  class pass {
  protected:
    state_map states;
    
  public:
    gprog* p;
    
    state* get_state(state_name n) {
      assert(states.count(n) > 0);
      return states[n];
    }

    virtual void exec(gprog* p);    
  };
  
  class state {
  protected:
    pass* t;

  public:
    virtual void update() { assert(false); }    

    template<typename T>
      T* get_state(state_name n) {
      state* s = t->get_state(n);
      return static_cast<T*>(s);
    }
  };

}

#endif
