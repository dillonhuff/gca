#ifndef GCA_SIM_MILL_H
#define GCA_SIM_MILL_H

#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_res.h"
#include "synthesis/cut.h"

namespace gca {

  class sim_mill_state {
  public:
    region& r;
    const mill_tool& t;
    double inc_size;
    sim_res res;
    
    sim_mill_state(region& rp, const mill_tool& tp) :
      r(rp), t(tp), inc_size(r.resolution / 2.0),
      res(GCA_SIM_OK) {}

    inline sim_res result() { return res; }

    void update_line(point sp, point ep) {
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
	r.update(c, t);
      }
    }
  };

  sim_res simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t);  
}

#endif
