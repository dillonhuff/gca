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
		     pass* tp,
		     value* initial_feedratep,
		     value* new_feedratep) :
    c(cp) {
      t = tp;
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
      p = c.mk_gprog();
    }

  feed_changer_state(context& cp,
		     pass* tp,
		     value* default_val,
		     value* initial_feedratep,
		     var* new_feedratep) :
    c(cp) {
      t = tp;
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
      p = c.mk_gprog();
      p->push_back(c.mk_assign(new_feedratep, default_val));
    }
    
    void update_default(instr& ist) {
      p->push_back(&ist);
    }

    void update_G1(move_instr& ist) {
      point kp = ist.pos();
      orientation orient = GCA_ABSOLUTE;//ist.is_abs() ? GCA_ABSOLUTE : GCA_RELATIVE;
      p->push_back(c.mk_G1(kp.x, kp.y, kp.z, new_feedrate, orient));
    }

  };

  class feed_changer : public pass {
  protected:
    feed_changer_state feed_s;

  public:
  feed_changer(context& c, value* initial_feedratep, value* new_feedratep) :
      feed_s(c, this, initial_feedratep, new_feedratep) {
      states[GCA_FEED_CHANGER_STATE] = &feed_s;
    }

  feed_changer(context& c, value* default_val, value* initial_feedratep, var* new_feedratep) :
      feed_s(c, this, default_val, initial_feedratep, new_feedratep) {
      states[GCA_FEED_CHANGER_STATE] = &feed_s;
    }
    
    virtual gprog* apply(gprog* p) {
      exec(p);
      return feed_s.p;
    }
  };

}

#endif
