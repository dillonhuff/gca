#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/polyline.h"

namespace gca {

  struct pocket {
    polyline outline;
    double start_depth;
    double end_depth;
    pocket(polyline outlinep,
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

  vector<polyline> pocket_2P5D_exterior(const pocket& pocket);

  vector<polyline> pocket_2P5D_interior(const pocket& pocket,
					double tool_diameter);
  vector<cut*> polyline_cuts(const polyline& p);

}

#endif
