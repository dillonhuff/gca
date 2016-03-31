#include "system/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/sim_mill.h"

namespace gca {

  double update_line(region& r, const mill_tool& t, point sp, point ep) {
    double volume_removed = 0.0;
    double inc_size = r.resolution / 2.0;
    point s = r.machine_coords_to_region_coords(sp);
    point e = r.machine_coords_to_region_coords(ep);
    if (!r.in_region(e, t)) {
      cout << "goes outside of region bounds" << endl;
      assert(false);
    }
    point d = e - s;
    point inc = inc_size * d.normalize();
    point c = s;
    while (c != e) {
      if (within_eps(c + inc, e, inc_size) || within_eps(c, e, inc_size)) {
	c = e;
      } else {
	c = c + inc;
      }
      volume_removed += r.update(c, t);
    }
    return volume_removed;
  }

  double simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t) {
    double volume_removed = 0.0;
    for (auto c : p) {
      assert(c->is_linear_cut());
      volume_removed += update_line(r, t, c->get_start(), c->get_end());
    }
    return volume_removed;
  }

}
