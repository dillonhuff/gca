#ifndef GCA_G0_MOVE_CHECKER_H
#define GCA_G0_MOVE_CHECKER_H

#include "basic_states.h"
#include "pass.h"

namespace gca {

  class move_checker_state : public per_instr_state {
  public:
    move_checker_state(pass* tp) {
      t = tp;
    }
    
    virtual void update_G0(instr* ist) {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      point diff = ps->diff;
      if (diff.z != 0 && (diff.x != 0 || diff.y != 0)) {
	warning_state* ws = get_state<warning_state>(GCA_WARNING_STATE);
	ws->add_warning("moves diagonally");
      }
    }
  };

  class g0_move_checker : public pass {
  protected:
    current_instr_state cis;
    position_state ps;
    warning_state s;
    move_checker_state ms;
    orientation_state orient_s;
    
  public:
  g0_move_checker(orientation def) :
    cis(this), ps(this, point(0, 0, 0)), ms(this), orient_s(this, def) {
      states[GCA_INSTR_STATE] = &cis;
      states[GCA_WARNING_STATE] = &s;
      states[GCA_POSITION_STATE] = &ps;
      states[GCA_MOVE_CHECKER_STATE] = &ms;
      states[GCA_ORIENTATION_STATE] = &orient_s;
    }
  };

}

#endif
