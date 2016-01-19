#ifndef GCA_SIM_MILL_H
#define GCA_SIM_MILL_H

#include "core/basic_states.h"
#include "simulators/mill_tool.h"
#include "simulators/region.h"

namespace gca {

  class sim_mill_state : public per_instr_state {
  public:
    region& r;
    const mill_tool& t;
  sim_mill_state(region& rp, const mill_tool& tp) : r(rp), t(tp) {}
    
  };

  void simulate_mill(gprog& p, region& r, const mill_tool& t);
}

#endif
