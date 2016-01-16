#ifndef GCA_PASS_H
#define GCA_PASS_H

#include <map>

#include "core/gprog.h"

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
    int i;
    int num_warns;

  pass() : num_warns(0) {}
    void add_warning(string s) {
      instr* current = (*p)[i];
      cout << "Warning at position: " << *current << " " << s << endl;
      num_warns++;      
    }

    inline void add_state(state_name n, state* s) { states[n] = s; }

    template<typename T>
    T* get_state(state_name n) {
      assert(states.count(n) > 0);
      return static_cast<T*>(states[n]);
    }

    virtual void exec(gprog* p);
  };
  
  class state {
  protected:
    pass* t;

  public:
    virtual void update(instr& ist) { assert(false); }

    void add_warning(string s) { t->add_warning(s); }

    template<typename T>
      T* get_state(state_name n) {
      return t->get_state<T>(n);
      //      return static_cast<T*>(s);
    }
  };

}

#endif
