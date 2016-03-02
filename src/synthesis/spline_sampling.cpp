
#include "synthesis/linear_cut.h"
#include "synthesis/spline_sampling.h"

namespace gca {
  
  void append_spline(const b_spline* s,
		     vector<cut*>& cuts) {
    // TODO: Come up with a better sampling strategy
    unsigned points_per_spline = 100;
    double last = 0.0;
    double inc = 1.0 / points_per_spline;
    for (unsigned i = 1; i < points_per_spline; i++) {
      double next = last + inc;
      cuts.push_back(linear_cut::make(s->eval(last), s->eval(next)));
      last = next;
    }
  }
  
  void append_splines(const vector<b_spline*>& splines,
		      vector<cut*>& cuts) {
    for (vector<b_spline*>::const_iterator it = splines.begin();
	 it != splines.end(); ++it) {
      append_spline(*it, cuts);
    }
  }
}
