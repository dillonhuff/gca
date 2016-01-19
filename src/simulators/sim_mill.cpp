#include "simulators/sim_mill.h"

namespace gca {

  void simulate_mill(gprog& p, region& r, const mill_tool& t) {
    pass ps;
    orientation_state orient_s(&ps, GCA_ABSOLUTE);
    ps.add_state(GCA_ORIENTATION_STATE, &orient_s);
    position_state pos_s(&ps, point(0, 0, 0));
    ps.add_state(GCA_POSITION_STATE, &pos_s);
    sim_mill_state sim_state(r, t);
    ps.add_state(GCA_SIM_MILL_STATE, &sim_state);
    ps.exec(&p);
    return;
  }

}
