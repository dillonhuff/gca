#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/cut.h"

namespace gca {

  struct pocket {
  private:
    vector<oriented_polygon> boundaries;
    vector<oriented_polygon> holes;

    double start_depth;
    
  public:

    vector<triangle> base;

    pocket(vector<oriented_polygon>& boundariesp,
	   vector<oriented_polygon>& holesp,
	   double start_depthp,
	   const vector<triangle>& basep) :
      boundaries(boundariesp),
      holes(holesp),
      start_depth(start_depthp),
      base(basep) {}

    inline const vector<oriented_polygon>& get_holes() const
    { return holes; }
    inline const vector<oriented_polygon>& get_boundaries() const
    { return boundaries; }
    inline double get_start_depth() const { return start_depth; }
    inline double get_end_depth() const { return min_z(base); }

    bool above_base(const point p) {
      for (auto t : base) {
	if (in_projection(t, p) && below(t, p)) { return false; }
      }
      return true;
    }
  };

  vector<polyline> deepen_polyline(const vector<double>& depths, const polyline& p);

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

  vector<polyline> rough_pocket(const pocket& pocket,
				double tool_radius,
				double cut_depth);

  vector<polyline> pocket_2P5D_interior(const pocket& pocket,
					double tool_diameter,
					double cut_depth);

  vector<cut*> polyline_cuts(const polyline& p);

  polyline compress_lines(const polyline& p, double tolerance);
}

#endif
