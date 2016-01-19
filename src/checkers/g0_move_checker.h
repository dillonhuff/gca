#ifndef GCA_G0_MOVE_CHECKER_H
#define GCA_G0_MOVE_CHECKER_H

#include "core/basic_states.h"
#include "core/pass.h"

namespace gca {

  class move_checker_state : public per_instr_state {
  public:
    move_checker_state(pass& tp)
      : per_instr_state(tp) {}
    
    virtual void update_G0(move_instr& ist) {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      point diff = ps->diff;
      if (diff.z != 0 && (diff.x != 0 || diff.y != 0)) {
	add_warning("moves diagonally");
      }
    }
  };

  int check_for_diagonal_G0_moves(gprog* p, orientation orient) {
    pass ps;
    position_state pos_s(ps, point(0, 0, 0));
    move_checker_state ms(ps);
    orientation_state orient_s(ps, orient);
    ps.add_state(GCA_POSITION_STATE, &pos_s);
    ps.add_state(GCA_MOVE_CHECKER_STATE, &ms);
    ps.add_state(GCA_ORIENTATION_STATE, &orient_s);
    ps.exec(p);
    return ps.num_warns;
  }

}

#endif
