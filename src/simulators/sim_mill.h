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
    double inc_size;
    
  sim_mill_state(pass& pp, region& rp, const mill_tool& tp) :
    per_instr_state(pp), r(rp), t(tp), inc_size(r.resolution / 2.0) {}

    point machine_coords_to_region_coords(point p) {
      assert(false);
    }

    point get_start() {
      cout << "Getting start" << endl;
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      cout << "Got ps" << endl;
      point machine_coord_start = ps->before;
      cout << "Got start" << endl;
      return machine_coords_to_region_coords(machine_coord_start);
    }
    point get_end() { assert(false); }

    void update_region(point c) {
    }

    void update_G1(move_instr& i) {
      point s = get_start();
      point e = get_end();
      point d = e - s;
      point inc = inc_size * d.normalize();
      point c = s;
      while (c != e) {
	if (within_eps(c + inc, e) || within_eps(c, e)) {
	  c = e;
	} else {
	  c = c + inc;
	}
	update_region(c);
      }
    }

    void update_default(instr& i) {
      cout << "Mill simulator error: Instruction " << i;
      cout << " is not supported" << endl;
      assert(false);
    }
  };

  void simulate_mill(gprog& p, region& r, const mill_tool& t);
}

#endif
