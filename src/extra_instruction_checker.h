#ifndef GCA_EXTRA_INSTRUCTION_CHECKER_H
#define GCA_EXTRA_INSTRUCTION_CHECKER_H

#include "basic_states.h"
#include "pass.h"

namespace gca {

  class orientation_checker : public state {
  public:
    orientation_checker(pass* tp) {
      t = tp;
    }

    virtual void update() {
      state* s = get_state(GCA_INSTR_STATE);
      current_instr_state* c = static_cast<current_instr_state*>(s);
      s = get_state(GCA_ORIENTATION_STATE);
      instr* ist = c->get_instr();
      orientation_state* os = static_cast<orientation_state*>(s);
      if (ist->is_G91() && os->current == GCA_RELATIVE) {
	state* s = get_state(GCA_WARNING_STATE);
	warning_state* ws = static_cast<warning_state*>(s);
	ws->add_warning("is not needed, relative coordinates are already turned on");
      }
    }
    
  };

  class extra_instruction_checker : public pass {
  protected:
    warning_state s;
    current_instr_state cis;
    orientation_state orient_state;
    orientation_checker orient_check;
    
  public:
    extra_instruction_checker()
      : cis(this), orient_state(this), orient_check(this) {
      states[GCA_INSTR_STATE] = &cis;
      states[GCA_WARNING_STATE] = &s;
      states[GCA_ORIENTATION_STATE] = &orient_state;
      states[GCA_ORIENTATION_CHECKER_STATE] = &orient_check;
    }
  };
}

#endif
