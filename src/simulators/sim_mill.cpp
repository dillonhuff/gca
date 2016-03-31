#include "system/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/sim_mill.h"

namespace gca {

  sim_res simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t) {
    sim_mill_state sim_state(r, t);
    for (auto c : p) {
      assert(c->is_linear_cut());
      sim_state.update_line(c->get_start(), c->get_end());
    }
    return sim_state.result();
  }

}
