#ifndef GCA_ABS_TO_REL_H
#define GCA_ABS_TO_REL_H

#include "core/basic_states.h"
#include "core/context.h"
#include "core/pass.h"

namespace gca {

  class abs_to_rel_state : public per_instr_state {
  public:
    gprog* p;

  abs_to_rel_state(pass& tp) :
    per_instr_state(tp) {
      p = mk_gprog();
    }

    void update_G0(move_instr& ist) {
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_ABSOLUTE) {
	position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
	point diff = ps->diff;
	p->push_back(g0_instr::make(diff));
      }
    }

    void update_G1(move_instr& ist) {
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_ABSOLUTE) {
	position_state* ps = get_state<position_state>(GCA_POSITION_STATE);
	point diff = ps->diff;
	p->push_back(g1_instr::make(diff.x, diff.y, diff.z, ist.feed_rate));
      }      
    }

    void update_default(instr& ist) {
      p->push_back(&ist);
    }

  };

  class abs_to_rel : pass {
  protected:
    abs_to_rel_state abs_to_rel_s;
    orientation_state orient_s;
    position_state pis;
    
  public:
  abs_to_rel(orientation def) :
    abs_to_rel_s(*this), orient_s(*this, def), pis(*this, point(0, 0, 0)) {
      states[GCA_POSITION_STATE] = &pis;
      states[GCA_ORIENTATION_STATE] = &orient_s;
      states[GCA_ABS_TO_REL_STATE] = &abs_to_rel_s;
    }
    
    virtual gprog* apply(gprog* p) {
      exec(p);
      return abs_to_rel_s.p;
    }
  };
  
}

#endif
