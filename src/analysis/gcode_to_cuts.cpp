#include "analysis/gcode_to_cuts.h"
#include "core/basic_states.h"
#include "synthesis/linear_cut.h"
#include "synthesis/safe_move.h"

namespace gca {
  
  vector<cut*> gcode_to_cuts(const gprog& p, const gcode_settings& settings) {
    pass ps;
    orientation_state orient_state(ps, settings.initial_coord_orient);
    position_state pos_state(ps, settings.initial_pos);
    ps.add_state(GCA_POSITION_STATE, &pos_state);
    ps.add_state(GCA_ORIENTATION_STATE, &orient_state);
    vector<cut*> cuts;
    for (ilist::const_iterator it = p.begin(); it != p.end(); ++it) {
      instr* i = *it;
      ps.update(i);
      cout << "i = " << *i << endl;
      if (i->is_G0()) {
	cuts.push_back(safe_move::make(pos_state.before, pos_state.after, settings.initial_tool));
      } else if (i->is_G1()) {
	g1_instr* g1i = static_cast<g1_instr*>(i);
	linear_cut* c = linear_cut::make(pos_state.before, pos_state.after, settings.initial_tool);
	c->feedrate = g1i->feed_rate;
	cuts.push_back(c);
      }
    }
    return cuts;
  }
  
}
