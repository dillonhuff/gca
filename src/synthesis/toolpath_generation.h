#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/polyline.h"

namespace gca {

  vector<polyline> deepen_polyline(vector<double> depths, const polyline& p);

  vector<double> cut_depths(double start_depth,
			    double end_depth,
			    double cut_depth);

  vector<polyline> tile_vertical(const vector<polyline>& ps,
				 double start_depth,
				 double end_depth,
				 double cut_depth);
  
}

#endif
