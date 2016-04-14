#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/polyline.h"

namespace gca {

  struct pocket_info_2P5D {
    polyline outline;
    double inc;
    double deg;
    int num_phases;
    double start_depth;
    double end_depth;
    double cut_depth;
    pocket_info_2P5D(polyline outlinep,
		     double incp,
		     double degp,
		     int num_phasesp,
		     double start_depthp,
		     double end_depthp,
		     double cut_depthp) :
      outline(outlinep),
      inc(incp),
      deg(degp),
      num_phases(num_phasesp),
      start_depth(start_depthp),
      end_depth(end_depthp),
      cut_depth(cut_depthp)
    {}

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
				    double degrees,
				    double inc);

  vector<polyline> pocket_2P5D_lines(const pocket_info_2P5D& pocket);

}

#endif
