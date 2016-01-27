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
    context& c;

  public:
    gprog* p;

  feed_changer_state(context& cp,
		     pass& tp,
		     value* initial_feedratep,
		     value* new_feedratep) :
    per_instr_state(tp), c(cp) {
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
      p = mk_gprog();
    }

  feed_changer_state(context& cp,
		     pass& tp,
		     value* default_val,
		     value* initial_feedratep,
		     var* new_feedratep) :
    per_instr_state(tp), c(cp) {
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
      p = mk_gprog();
      p->push_back(mk_assign(new_feedratep, default_val));
    }
    
    void update_default(instr& ist) {
      p->push_back(&ist);
    }

    void update_G1(move_instr& ist) {
      point kp = ist.pos();
      p->push_back(mk_G1(kp.x, kp.y, kp.z, new_feedrate));
    }

  };

  gprog* change_feeds(context& c, gprog* p, value* initial_feedratep, value* new_feedratep) {
    pass ps;
    feed_changer_state feed_s(c, ps, initial_feedratep, new_feedratep);
    ps.add_state(GCA_FEED_CHANGER_STATE, &feed_s);
    ps.exec(p);
    return ps.get_state<feed_changer_state>(GCA_FEED_CHANGER_STATE)->p;
  }

  gprog* generalize_feeds(context& c, gprog* p, value* default_val, value* initial_feedratep, var* new_feedratep) {
    pass ps;
    feed_changer_state feed_s(c, ps, default_val, initial_feedratep, new_feedratep);
    ps.add_state(GCA_FEED_CHANGER_STATE, &feed_s);
    ps.exec(p);
    return ps.get_state<feed_changer_state>(GCA_FEED_CHANGER_STATE)->p;
  }
  
}

#endif
