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

    virtual gprog* apply(gprog* p, orientation def) {
      point last_pos = p->last_position();
      point diff = (start + shift) - last_pos;
      gprog* rp;
      abs_to_rel atr(def);
      rp = atr.apply(p);
      gprog* new_p = mk_gprog();
      new_p->push_back(mk_G91());
      for (int i = 0; i < num_copies; i++) {
	int j = 0;
	int num_copied = 0;
	while (j < rp->size() && !((*rp)[j]->is_end_instr())) {
	  instr* cpy = mk_instr_cpy((*rp)[j]);
	  new_p->push_back(cpy);
	  num_copied++;
	  j++;
	}
	if (i != num_copies - 1 && num_copied > 0) {
	  new_p->push_back(mk_G0(0, 0, diff.z));
	  new_p->push_back(mk_G0(diff.x, diff.y, 0));
	}
      }
      new_p->push_back(mk_m2_instr());
      return new_p;
    }

  };
}

#endif
