#ifndef GCA_TILER_H
#define GCA_TILER_H

#include "abs_to_rel.h"

namespace gca {
  class tiler {
  protected:
    int num_copies;
    point start, shift;
    
  public:
  tiler(int nc, point s, point v) :
    num_copies(nc), start(s), shift(v) {}

    virtual gprog* apply(context& c, gprog* p, orientation def) {
      point last_pos = p->last_position();
      point diff = (start + shift) - last_pos;
      gprog* rp;
      if (p->all_abs()) {
	abs_to_rel atr(c, def);
	rp = atr.apply(c, p);
      } else {
	rp = p;
      }
      assert(rp->all_rel());
      gprog* new_p = c.mk_gprog();
      new_p->push_back(c.mk_G91());
      for (int i = 0; i < num_copies; i++) {
	int j = 0;
	int num_copied = 0;
	while (j < rp->size() && !((*rp)[j]->is_end_instr())) {
	  instr* cpy = c.mk_instr_cpy((*rp)[j]);
	  new_p->push_back(cpy);
	  num_copied++;
	  j++;
	}
	if (i != num_copies - 1 && num_copied > 0) {
	  new_p->push_back(c.mk_G0(0, 0, diff.z, GCA_RELATIVE));
	  new_p->push_back(c.mk_G0(diff.x, diff.y, 0, GCA_RELATIVE));
	}
      }
      new_p->push_back(c.mk_minstr(2));
      return new_p;
    }

  };
}
#endif
