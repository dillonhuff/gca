#include "analysis/gcode_to_cuts.h"
#include "core/basic_states.h"
#include "core/callback.h"
#include "synthesis/linear_cut.h"
#include "synthesis/safe_move.h"

namespace gca {

  struct cuts_callback : public per_instr_callback<cut*> {
    pass ps;
    const gcode_settings& settings;    
    orientation_state orient_state;
    position_state pos_state;

    cuts_callback(const gcode_settings& psettings) :
      settings(psettings),
      orient_state(ps, settings.initial_coord_orient),
      pos_state(ps, settings.initial_pos) {
      ps.add_state(GCA_POSITION_STATE, &pos_state);
      ps.add_state(GCA_ORIENTATION_STATE, &orient_state);
    }

    cut* call_default(gprog* p, int i, instr* is) {
      return NULL;
    }

    cut* call_G0(gprog* p, int i, g0_instr* is) {
      return safe_move::make(pos_state.before, pos_state.after, settings.initial_tool);
    }

    cut* call_G1(gprog* p, int i, g1_instr* is) {
      linear_cut* c = linear_cut::make(pos_state.before, pos_state.after, settings.initial_tool);
      c->feedrate = is->feed_rate;
      return c;
    }

    void update(instr* i) {
      ps.update(i);
    }
  };
  
  vector<cut*> gcode_to_cuts(gprog& p, const gcode_settings& settings) {
    cuts_callback c(settings);
    vector<cut*> cuts;
    int i = 0;
    for (ilist::const_iterator it = p.begin(); it != p.end(); ++it) {
      c.update(*it);
      cut* ct = c(&p, i, *it);
      if (ct != NULL) {
	cuts.push_back(ct);
      }
      i++;
    }
    return cuts;
  }
  
}
