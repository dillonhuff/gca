#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/polyline.h"

namespace gca {

  struct pocket_info_2P5D {
    polyline outline;
    double start_depth;
    double end_depth;
    pocket_info_2P5D(polyline outlinep,
		     double start_depthp,
		     double end_depthp) :
      outline(outlinep),
      start_depth(start_depthp),
      end_depth(end_depthp) {}
  };

  vector<polyline> deepen_polyline(vector<double> depths, const polyline& p);

  vector<double> cut_depths(double start_depth,
			    double end_depth,
			    double cut_depth);

  vector<polyline> tile_vertical(const vector<polyline>& ps,
				 double start_depth,
				 double end_depth,
				 double cut_depth);

  vector<polyline> repeated_offsets(const polyline& p,
				    int num_repeats,
				    offset_dir d,
				    double inc);

  vector<polyline> pocket_2P5D_lines(const pocket_info_2P5D& pocket);

  vector<cut*> polyline_cuts(const polyline& p);

}

#endif
