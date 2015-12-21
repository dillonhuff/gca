#include <math.h>

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

  cut* sink_cut(context& c, cut* s, double l) {
    double xd = s->end.x - s->start.x;
    if (xd == 0) {
      return c.mk_cut(point(s->start.x, s->start.y - l, 0), s->start);
    }
    double m = (s->end.y - s->start.y) / xd;
    double a = sqrt((l*l) / (1.0 + m*m));
    double b = m*a;
    return c.mk_cut(point(s->start.x - a, s->start.y - b, 0), s->start);
  }
  
}
