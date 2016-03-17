#ifndef GCA_SPLINE_SAMPLING_H
#define GCA_SPLINE_SAMPLING_H

#include "geometry/b_spline.h"
#include "geometry/polyline.h"
#include "synthesis/cut.h"
#include "synthesis/toolpath.h"

namespace gca {
  
  void append_splines(const vector<b_spline*>& splines,
		      vector<polyline>& polys);
  
}

#endif
