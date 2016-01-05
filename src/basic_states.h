#ifndef GCA_BASIC_STATES_H
#define GCA_BASIC_STATES_H

#include "state.h"
#include "pass.h"

#define GCA_CURRENT_INSTR_STATE 0
#define GCA_WARNING_STATE 1

namespace gca {

  class current_instr_state : public state {
  protected:
    pass* t;
    int i;
  public:
  current_instr_state(pass* tp) :
    t(tp), i(0) {
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

}

#endif
