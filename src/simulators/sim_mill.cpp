#include "system/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/sim_mill.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"

namespace gca {

  void check_region_bounds(const point e, const region& r, const mill_tool& t) {
    if (!r.in_region(e, t)) {
      cout << e << " goes outside of region bounds" << endl;
      cout << "Region height: " << r.height << endl;
      cout << "Region x len: " << r.x_len << endl;
      cout << "Region y len: " << r.y_len << endl;
      cout << "Machine x offset: " << r.machine_x_offset << endl;
      cout << "Machine y offset: " << r.machine_y_offset << endl;
      cout << "Machine z offset: " << r.machine_z_offset << endl;
      assert(false);
    }
  }

  double update_cut(region& r, const mill_tool& t, const cut& c) {
    double volume_removed = 0.0;
    double d = r.resolution;
    int num_points = (c.length() / d) + 1;
    for (int i = 0; i < num_points; i++) {
      double tp = static_cast<double>(i) / static_cast<double>(num_points);
      point e = r.machine_coords_to_region_coords(c.value_at(tp));
      check_region_bounds(e, r, t);
      volume_removed += r.update(e, t);
    }
    return volume_removed;
  }
  
  double simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t) {
    double volume_removed = 0.0;
    for (auto c : p) {
      volume_removed += update_cut(r, t, *c);
    }
    return volume_removed;
  }

}
