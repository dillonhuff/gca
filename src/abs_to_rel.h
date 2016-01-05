#ifndef GCA_ABS_TO_REL_H
#define GCA_ABS_TO_REL_H

#include "basic_states.h"
#include "context.h"
#include "pass.h"

#define GCA_ABS_TO_REL_STATE 2003

namespace gca {

  class abs_to_rel_state : public per_instr_state {
  public:
    gprog* p;
    context& c;

  abs_to_rel_state(context& cp, pass* tp) :
    c(cp) {
      t = tp;
      p = c.mk_gprog();
    }

    void update_G0(instr* ist) {
      state* s = get_state(GCA_ORIENTATION_STATE);
      orientation_state* os = static_cast<orientation_state*>(s);
      if (os->current == GCA_ABSOLUTE) {
	position_state* ps = static_cast<position_state*>(get_state(GCA_POSITION_STATE));
	point diff = ps->diff;
	p->push_back(c.mk_G0(diff, GCA_RELATIVE));
      }
    }

    void update_G1(instr* ist) {
      state* s = get_state(GCA_ORIENTATION_STATE);
      orientation_state* os = static_cast<orientation_state*>(s);
      if (os->current == GCA_ABSOLUTE) {
	position_state* ps = static_cast<position_state*>(get_state(GCA_POSITION_STATE));
	point diff = ps->diff;
	p->push_back(c.mk_G1(diff.x, diff.y, diff.z, ist->feed_rate, GCA_RELATIVE));
      }      
    }

    void update_default(instr* ist) {
      p->push_back(ist);
    }

  };

  class abs_to_rel : pass {
  protected:
    abs_to_rel_state abs_to_rel_s;
    current_instr_state cis;
    orientation_state orient_s;
    position_state pis;
    
  public:
  abs_to_rel(context& c, orientation def) :
    abs_to_rel_s(c, this), cis(this), orient_s(this, def), pis(this, point(0, 0, 0)) {
      states[GCA_INSTR_STATE] = &cis;
      states[GCA_POSITION_STATE] = &pis;
      states[GCA_ORIENTATION_STATE] = &orient_s;
      states[GCA_ABS_TO_REL_STATE] = &abs_to_rel_s;
    }
    
    virtual gprog* apply(context& c, gprog* p) {
      exec(p);
      return abs_to_rel_s.p;
    }
  };
  
}

#endif
