#include "core/context.h"
#include "synthesis/spline_sampling.h"

namespace gca {
  
  void append_spline(const b_spline* s,
		     vector<vector<cut*> >& cut_groups) {
    unsigned points_per_spline = 100;
    vector<cut*> spline_cuts;
    double last = 0.0;
    double inc = 1.0 / points_per_spline;
    for (unsigned i = 1; i < points_per_spline; i++) {
      double next = last + inc;
      spline_cuts.push_back(mk_linear_cut(s->eval(last), s->eval(next)));
      last = next;
    }
    cut_groups.push_back(spline_cuts);
  }

  void append_splines(const vector<b_spline*>& splines,
		      vector<cut_group>& cut_groups) {
    for (unsigned i = 0; i < splines.size(); i++) {
      append_spline(splines[i], cut_groups);
    }
  }

}
