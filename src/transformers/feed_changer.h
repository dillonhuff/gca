#ifndef GCA_FEED_CHANGER_H
#define GCA_FEED_CHANGER_H

#include "core/basic_states.h"
#include "core/context.h"
#include "core/pass.h"

#define GCA_FEED_CHANGER_STATE 2001

namespace gca {

  class feed_changer_state : public per_instr_state {
  protected:
    value* initial_feedrate;
    value* new_feedrate;

  public:
    gprog* p;

  feed_changer_state(pass& tp,
		     value* initial_feedratep,
		     value* new_feedratep) :
    per_instr_state(tp) {
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
      p = gprog::make();
    }

  feed_changer_state(pass& tp,
		     value* default_val,
		     value* initial_feedratep,
		     var* new_feedratep) :
    per_instr_state(tp) {
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
      p = gprog::make();
      p->push_back(mk_assign(new_feedratep, default_val));
    }
    
    void update_default(instr& ist) {
      p->push_back(&ist);
    }

    void update_G1(move_instr& ist) {
      point kp = ist.pos();
      p->push_back(g1_instr::make(kp.x, kp.y, kp.z, new_feedrate));
    }

  };

  gprog* change_feeds(gprog* p, value* initial_feedratep, value* new_feedratep) {
    pass ps;
    feed_changer_state feed_s(ps, initial_feedratep, new_feedratep);
    ps.add_state(GCA_FEED_CHANGER_STATE, &feed_s);
    ps.exec(p);
    return ps.get_state<feed_changer_state>(GCA_FEED_CHANGER_STATE)->p;
  }

  gprog* generalize_feeds(gprog* p, value* default_val, value* initial_feedratep, var* new_feedratep) {
    pass ps;
    feed_changer_state feed_s(ps, default_val, initial_feedratep, new_feedratep);
    ps.add_state(GCA_FEED_CHANGER_STATE, &feed_s);
    ps.exec(p);
    return ps.get_state<feed_changer_state>(GCA_FEED_CHANGER_STATE)->p;
  }
  
}

#endif
