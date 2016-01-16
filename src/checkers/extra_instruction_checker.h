#ifndef GCA_EXTRA_INSTRUCTION_CHECKER_H
#define GCA_EXTRA_INSTRUCTION_CHECKER_H

#include "core/basic_states.h"
#include "core/pass.h"

namespace gca {

  int check_for_extra_instructions(gprog* p, orientation orient) {
    pass ps;
    orientation_state orient_s(&ps, orient);
    orientation_checker orient_check(&ps);
    ps.add_state(GCA_ORIENTATION_STATE, &orient_s);
    ps.add_state(GCA_ORIENTATION_CHECKER_STATE, &orient_check);
    ps.exec(p);
    return ps.num_warns;
  }
  
}

#endif
