#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/box.h"
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

  std::vector<polyline> deepen_polyline(const std::vector<double>& depths,
					const polyline& p);

  std::vector<double> cut_depths(double start_depth,
				 double end_depth,
				 double cut_depth);

  std::vector<polyline> tile_vertical(const std::vector<polyline>& ps,
				      double start_depth,
				      double end_depth,
				      double cut_depth);

  std::vector<polyline> repeated_offsets(const polyline& p,
					 int num_repeats,
					 offset_dir d,
					 double inc);

  std::vector<polyline> rough_pocket(const pocket& pocket,
				     double tool_radius,
				     double cut_depth);

  std::vector<polyline> rough_pockets(const std::vector<pocket>& pockets,
				      double tool_radius,
				      double cut_depth);
  
  std::vector<polyline> pocket_2P5D_interior(const pocket& pocket,
					     double tool_diameter,
					     double cut_depth);

  std::vector<cut*> polyline_cuts(const polyline& p);

  polyline compress_lines(const polyline& p, double tolerance);

  std::vector<polyline> rough_box(const box b,
				  double tool_radius,
				  double cut_depth);

  std::vector<polyline> drop_sample(const std::vector<triangle>& triangles,
				    double tool_radius);
  
}

#endif
