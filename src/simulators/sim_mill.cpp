#include "simulators/sim_mill.h"

namespace gca {

  sim_res simulate_mill(gprog& p, region& r, const mill_tool& t) {
    pass ps;
    orientation_state orient_s(ps, GCA_ABSOLUTE);
    position_state pos_s(ps, point(0, 0, 0));
    sim_mill_state sim_state(ps, r, t);
    ps.add_state(GCA_POSITION_STATE, &pos_s);
    ps.add_state(GCA_ORIENTATION_STATE, &orient_s);    
    ps.add_state(GCA_SIM_MILL_STATE, &sim_state);
    ps.exec(&p);
    return sim_state.result();
  }

}
