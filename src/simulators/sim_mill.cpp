#include "system/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/sim_mill.h"
#include "synthesis/circular_arc.h"

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

  double update_arc(region& r, const mill_tool& t, circular_arc* ar) {
    double volume_removed = 0.0;
    // TODO: Get rid of 100 magic number
    point s;
    for (int i = 0; i < 100; i++) {
      double tp = static_cast<double>(i) / static_cast<double>(100);
      point e = r.machine_coords_to_region_coords(ar->value(tp));
      if (!r.in_region(e, t)) {
	cout << "goes outside of region bounds" << endl;
	assert(false);
      }
      volume_removed += r.update(e, t);
    }
    return volume_removed;
  }

  double simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t) {
    double volume_removed = 0.0;
    for (auto c : p) {
      if (c->is_linear_cut()) {
	volume_removed += update_line(r, t, c->get_start(), c->get_end());
      } else if (c->is_circular_arc()) {
	circular_arc* ca = static_cast<circular_arc*>(c);
	volume_removed += update_arc(r, t, ca);
      }
    }
    return volume_removed;
  }

}
