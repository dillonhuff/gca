#ifndef GCA_REL_TO_ABS_H
#define GCA_REL_TO_ABS_H

#include "core/basic_states.h"
#include "core/context.h"
#include "core/pass.h"

namespace gca {

  class rel_to_abs_state : public per_instr_state {
  public:
    gprog* p;
    context& c;

  rel_to_abs_state(context& cp, pass* tp) :
    c(cp) {
      t = tp;
      p = c.mk_gprog();
    }

    void update_G0(move_instr& ist) {
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_RELATIVE) {
	position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
	point after = ps->after;
	p->push_back(c.mk_G0(after, GCA_ABSOLUTE));
      }
    }

    void update_G1(move_instr& ist) {
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_RELATIVE) {
	position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
	point after = ps->after;
	p->push_back(c.mk_G1(after.x, after.y, after.z, ist.feed_rate, GCA_ABSOLUTE));
      }
    }

    void update_G91(instr& ist) {}
    void update_default(instr& ist) { p->push_back(&ist); }

  };

  class rel_to_abs : pass {
  protected:
    rel_to_abs_state rel_to_abs_s;
    orientation_state orient_s;
    position_state pis;
    
  public:
  rel_to_abs(context& c, orientation def) :
    rel_to_abs_s(c, this), orient_s(this, def), pis(this, point(0, 0, 0)) {
      states[GCA_POSITION_STATE] = &pis;
      states[GCA_ORIENTATION_STATE] = &orient_s;
      states[GCA_REL_TO_ABS_STATE] = &rel_to_abs_s;
    }
    
    virtual gprog* apply(context& c, gprog* p) {
      exec(p);
      return rel_to_abs_s.p;
    }
  };
  
}

#endif
