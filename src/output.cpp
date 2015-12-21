#include <cmath>

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
    double yd = s->end.y - s->start.y;
    double x_pos = xd > 0;
    double y_pos = yd > 0;
    if (xd == 0) {
      return c.mk_cut(point(s->start.x, s->start.y - l, 0), s->start);
    }
    double m = yd / xd;
    double a = sqrt((l*l) / (1.0 + m*m));
    double b = m*a;
    double xs, ys;
    if (x_pos) {
      xs = s->start.x - abs(a);
    } else {
      xs = s->start.x + abs(a);
    }
    if (y_pos) {
      ys = s->start.y - abs(b);
    } else {
      ys = s->start.y + abs(b);
    }
    return c.mk_cut(point(xs, ys, 0), s->start);
  }
  
}
