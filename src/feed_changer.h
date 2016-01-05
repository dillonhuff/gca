#ifndef GCA_FEED_CHANGER_H
#define GCA_FEED_CHANGER_H

#include "basic_states.h"
#include "context.h"
#include "pass.h"

#define GCA_FEED_CHANGER_STATE 2001

namespace gca {

  class feed_changer_state : public per_instr_state {
  protected:
    double initial_feedrate, new_feedrate;
    context& c;

  public:
    gprog* p;
    
  feed_changer_state(context& cp,
		     pass* tp,
		     double initial_feedratep,
		     double new_feedratep) :
    c(cp) {
      t = tp;
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
      p = c.mk_gprog();
    }

    void update_default(instr* ist) {
      p->push_back(ist);
    }

    void update_G1(instr* ist) {
      point kp = ist->pos();
      orientation orient = ist->is_abs() ? GCA_ABSOLUTE : GCA_RELATIVE;
      p->push_back(c.mk_G1(kp.x, kp.y, kp.z, new_feedrate, orient));
    }

  };

  class feed_changer : public pass {
  protected:
    current_instr_state cis;
    feed_changer_state feed_s;

  public:
  feed_changer(context& c, double initial_feedratep, double new_feedratep) :
    cis(this),
      feed_s(c, this, initial_feedratep, new_feedratep) {
      states[GCA_INSTR_STATE] = &cis;
      states[GCA_FEED_CHANGER_STATE] = &feed_s;
    }

    virtual gprog* apply(context& c, gprog* p) {
      exec(p);
      return feed_s.p;
    }
  };

}

#endif
