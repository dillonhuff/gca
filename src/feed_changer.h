#ifndef GCA_FEED_CHANGER_H
#define GCA_FEED_CHANGER_H

#include "context.h"

namespace gca {

  class feed_changer {
  protected:
    double initial_feedrate, new_feedrate;
    
  public:
    feed_changer(double initial_feedratep, double new_feedratep) {
      initial_feedrate = initial_feedratep;
      new_feedrate = new_feedratep;
    }

    virtual gprog* apply(context& c, gprog* p) {
      gprog* n = c.mk_gprog();
      for (ilist::iterator it = p->begin();
	   it != p->end(); ++it) {
	instr k = **it;
	if (k.is_G1()) {
	  point kp = k.pos();
	  orientation orient = k.is_abs() ? GCA_ABSOLUTE : GCA_RELATIVE;
	  n->push_back(c.mk_G1(kp.x, kp.y, kp.z, new_feedrate, orient));
	} else {
	  n->push_back(&k);
	}
      }
      return n;
    }
  };

}

#endif

