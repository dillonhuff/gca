#ifndef GCA_BASIC_STATES_H
#define GCA_BASIC_STATES_H

#include "state.h"
#include "pass.h"

#define GCA_INSTR_STATE 0
#define GCA_WARNING_STATE 1

namespace gca {

  class current_instr_state : public state {
  protected:
    int i;
  public:
    current_instr_state(pass* tp) {
      t = tp;
      i = 0;
    }

    virtual void update() {
      i++;
    }

    instr* next_instr() { return (*(t->p))[i]; }
  };
  
  class warning_state : public state {
  public:
    int num_warnings() { return 0; }
    virtual void update() {}
  };

  class gca_position_state : public state {
  protected:
    point before, after, diff;
  public:
    gca_position_state(pass* tp) {
      t = tp;
    }
    
    virtual void update() {
      state* s = get_state(GCA_INSTR_STATE);
      current_instr_state* c = static_cast<current_instr_state*>(s);
      //      s = get_state(GCA_CURRENT_ORIENTATION_STATE);
    }
  };

}

#endif
