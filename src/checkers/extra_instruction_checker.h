#ifndef GCA_EXTRA_INSTRUCTION_CHECKER_H
#define GCA_EXTRA_INSTRUCTION_CHECKER_H

#include "core/basic_states.h"
#include "core/pass.h"

namespace gca {

  class extra_instruction_checker : public pass {
  protected:
    warning_state s;
    orientation_state orient_state;
    orientation_checker orient_check;
    
  public:
    extra_instruction_checker(orientation def)
      : s(this), orient_state(this, def), orient_check(this) {
      states[GCA_WARNING_STATE] = &s;
      states[GCA_ORIENTATION_STATE] = &orient_state;
      states[GCA_ORIENTATION_CHECKER_STATE] = &orient_check;
    }
  };
}

#endif
