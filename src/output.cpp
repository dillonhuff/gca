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
	  p->push_back(c.mk_G1(cuts[i]->start.x, cuts[i]->start.y, cuts[i]->start.z));
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

  gprog* gcode_for_surface(context& c, vector<cut*>& cuts) {
    double z_eps = 0.2;
    gprog* p = c.mk_gprog();
    instr* last = NULL;
    for (int i = 0; i < cuts.size(); i++) {
      if (!(i > 0 && within_eps(cuts[i-1]->end.z, cuts[i]->start.z))) {
	if (i > 0) {
	  p->push_back(c.mk_G0(cuts[i-1]->end.x, cuts[i-1]->end.y, z_eps));
	}
	p->push_back(c.mk_G0(cuts[i]->start.x, cuts[i]->start.y, z_eps));
	if (cuts[i]->start.z != 0.0) {
	  p->push_back(c.mk_G1(cuts[i]->start.x, cuts[i]->start.y, cuts[i]->start.z));
	}
      } else {
	if (cuts[i]->start.z != 0.0) {
	  p->push_back(c.mk_G1(cuts[i]->start.x, cuts[i]->start.y, cuts[i]->start.z));
	}	
      }
      last = c.mk_G1(cuts[i]->end.x, cuts[i]->end.y, cuts[i]->end.z);
      p->push_back(last);
    }
    if (last != NULL) {
      p->push_back(c.mk_G0(last->x, last->y, z_eps));
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

  vector<cut*> surface_cuts(context &c,
			    point left, point right,
			    point shift, int num_cuts) {
    vector<cut*> cuts;
    point c_left = left;
    point c_right = right;
    for (int i = 1; i <= num_cuts; i++) {
      cuts.push_back(c.mk_cut(c_left, c_right));
      // Reverse cut directions
      point temp = c_left;
      c_left = c_right + shift;
      c_right = temp + shift;
    }
    return cuts;
  }


  vector<cut*> two_pass_surface(double coarse_depth, double finish_inc,
				double cutter_width,
				double x_s, double x_e, double y,
				double width) {
    double inc = cutter_width*(2.0/3.0);
    int num_cuts = ceil(width / inc);
    point coarse_start(x_s, y, coarse_depth);
    point coarse_end(x_e, y, coarse_depth);
    point shift(0, inc, 0);
    double finish_depth = coarse_depth + finish_inc;
    point finish_start(x_s, y, finish_depth);
    point finish_end(x_e, y, finish_depth);
    context c;
    vector<cut*> cuts1 = surface_cuts(c, coarse_start, coarse_end,
				      shift, num_cuts);
    vector<cut*> cuts2 = surface_cuts(c, finish_start, finish_end,
				      shift, num_cuts);
    cuts1.insert(cuts1.end(), cuts2.begin(), cuts2.end());
    return cuts1;
  }
  
}
