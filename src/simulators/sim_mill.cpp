#include "core/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/sim_mill.h"

namespace gca {

  sim_res simulate_mill(const vector<line> p, region& r, const mill_tool& t) {
    sim_mill_state sim_state(r, t);
    for (auto l : p) {
      sim_state.update_line(l.start, l.end);
    }
    return sim_state.result();
  }

}
