#include "output.h"

namespace gca {
  
  gprog* gcode_for_cuts(context& c, vector<cut*>& cuts) {
    gprog* p = c.mk_gprog();
    instr* last = NULL;
    for (int i = 0; i < cuts.size(); i++) {
      if (!(i > 0 && within_eps(cuts[i-1]->end, cuts[i]->start))) {
	p->push_back(c.mk_G0(cuts[i]->start.x, cuts[i]->start.y, cuts[i]->start.z));
      }
      last = c.mk_G1(cuts[i]->end.x, cuts[i]->end.y, cuts[i]->end.z);
      p->push_back(last);
    }
    if (last != NULL) {
      p->push_back(c.mk_G0(last->x, last->y, 0.0));
    }
    p->push_back(c.mk_G0(0, 0, 0));
    p->push_back(c.mk_minstr(2));
    return p;
  }
  
}
