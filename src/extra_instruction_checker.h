#ifndef GCA_EXTRA_INSTRUCTION_CHECKER_H
#define GCA_EXTRA_INSTRUCTION_CHECKER_H

#include "basic_states.h"
#include "pass.h"

namespace gca {

  class extra_instruction_checker : public pass {
  protected:
    warning_state s;
    current_instr_state cis;
    orientation_state orient_state;
    
  public:
    extra_instruction_checker()
      : cis(this), orient_state(this) {
      warning_state sp;
      s = sp;
      states[GCA_INSTR_STATE] = &cis;
      states[GCA_WARNING_STATE] = &s;
      states[GCA_ORIENTATION_STATE] = &orient_state;
    }
  };
}

#endif
