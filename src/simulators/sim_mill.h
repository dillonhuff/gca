#ifndef GCA_SIM_MILL_H
#define GCA_SIM_MILL_H

#include "core/basic_states.h"
#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_res.h"

namespace gca {

  class sim_mill_state : public per_instr_state {
  public:
    region& r;
    const mill_tool& t;
    double inc_size, machine_x_offset, machine_y_offset;
    
  sim_mill_state(pass& pp, region& rp, const mill_tool& tp) :
    per_instr_state(pp), r(rp), t(tp), inc_size(r.resolution / 2.0) {}

    point get_start() {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      point machine_coord_start = ps->before;
      return r.machine_coords_to_region_coords(machine_coord_start);
    }

    point get_end() {
      position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
      point machine_coord_end = ps->after;
      return r.machine_coords_to_region_coords(machine_coord_end);
    }

    void update_line() {
      point s = get_start();
      point e = get_end();
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

    void update_G1(move_instr& i) { update_line(); }
    void update_G0(move_instr& i) { update_line(); }
    void update_G91(instr& i) {}
    void update_M2(instr& i) {}

    void update_default(instr& i) {
      cout << "Mill simulator error: Instruction " << i;
      cout << " is not supported" << endl;
      assert(false);
    }
  };

  sim_res simulate_mill(gprog& p, region& r, const mill_tool& t);
}

#endif
