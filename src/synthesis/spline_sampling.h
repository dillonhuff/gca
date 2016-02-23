#ifndef GCA_SPLINE_SAMPLING_H
#define GCA_SPLINE_SAMPLING_H

#include "geometry/b_spline.h"
#include "synthesis/cut.h"
#include "synthesis/toolpath.h"

namespace gca {
  
    void append_spline(const b_spline* s,
		       vector<vector<cut*> >& cut_groups);

    void append_splines(const vector<b_spline*>& splines,
			vector<cut_group>& cut_groups);

}

#endif
