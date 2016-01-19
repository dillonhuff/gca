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
    double inc_size, machine_x_offset, machine_y_offset;
    
  sim_mill_state(pass& pp, region& rp, const mill_tool& tp) :
    per_instr_state(pp), r(rp), t(tp), inc_size(r.resolution / 2.0),
      machine_x_offset(-5), machine_y_offset(-5) {}

    // TODO: Generalize to allow for user specification of
    // machine and region coordinates
    point machine_coords_to_region_coords(point p) {
      return point(p.x + machine_x_offset,
		   p.y + machine_y_offset,
		   r.height - p.z);
    }

    point get_start() {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      point machine_coord_start = ps->before;
      return machine_coords_to_region_coords(machine_coord_start);
    }

    point get_end() {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      point machine_coord_end = ps->after;
      return machine_coords_to_region_coords(machine_coord_end);
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
	r.update(c, t);
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
