#include <cmath>

#include "output.h"

namespace gca {
  
  gprog* gcode_for_cuts(context& c, vector<cut*>& cuts) {
    gprog* p = c.mk_gprog();
    instr* last = NULL;
    for (int i = 0; i < cuts.size(); i++) {
      if (!(i > 0 && within_eps(cuts[i-1]->end, cuts[i]->start))) {
	if (i > 0) {
	  p->push_back(c.mk_G0(cuts[i-1]->end.x, cuts[i-1]->end.y, 0.0));
	}
	p->push_back(c.mk_G0(cuts[i]->start.x, cuts[i]->start.y, 0.0));
	if (cuts[i]->start.z != 0.0) {
	  p->push_back(c.mk_G0(cuts[i]->start.x, cuts[i]->start.y, cuts[i]->start.z));
	}
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
      if (y_pos) {
	return c.mk_cut(point(s->start.x, s->start.y - l, 0), s->start);
      } else {
	return c.mk_cut(point(s->start.x, s->start.y + l, 0), s->start);
      }
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
      cout << "Y POSITIVE" << endl;
      cout << "y = " << yd;
      ys = s->start.y - abs(b);
    } else {
      cout << "Y NEGATIVE" << endl;
      ys = s->start.y + abs(b);
    }
    return c.mk_cut(point(xs, ys, 0), s->start);
  }

  void insert_sink_cuts(context& c, double l, vector<cut*>& cuts, vector<cut*>& dest) {
    for (int i = 0; i < cuts.size(); i++) {
      dest.push_back(sink_cut(c, cuts[i], l));
      dest.push_back(cuts[i]);
    }
  }
  
}
