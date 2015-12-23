#ifndef GCA_ABS_TO_REL_H
#define GCA_ABS_TO_REL_H

#include "context.h"

namespace gca {

  class abs_to_rel {
  protected:
    instr* abs_rel(context& c, instr* ist, point prev, point start) {
      point current = ist->pos();
      point next = current - prev - start;
      instr* next_ist;
      if (ist->v == 0) {
	next_ist = c.mk_G0(next.x, next.y, next.z, GCA_RELATIVE);
      } else if (ist->v == 1) {
	next_ist = c.mk_G1(next.x, next.y, next.z, ist->feed_rate, GCA_RELATIVE);
      } else {
	assert(false);
      }
      assert(next_ist->is_rel());
      cout << "next_is is relative" << endl;
      return next_ist;
    }
  public:
    virtual gprog* apply(context& c, gprog* p) {
      gprog* g = c.mk_gprog();
      point start(0, 0, 0);
      point prev(0, 0, 0);
      for (unsigned int i = 0; i < p->size(); i++) {
	instr* ist = (*p)[i];
	if (ist->is_G()) {
	  g->push_back(abs_rel(c, ist, prev, start));
	  prev = ist->pos();
	} else {
	  g->push_back(c.mk_inverted_orientation(ist));
	}
      }
      return g;
    }
    
  };
  
}

#endif
