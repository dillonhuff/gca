#ifndef GCA_ALIGN_BLADE_H
#define GCA_ALIGN_BLADE_H

#include "geometry/point.h"

namespace gca {

  class circular_arc {
  public:
    point start;
    point end;
    point start_offset;
  circular_arc(point sp, point ep, point so) : start(sp), end(ep), start_offset(so) {}

    inline point center() const { return start + start_offset; }
  };

  
  double angle_between(point u, point v);

  circular_arc align_coords(point desired_dir,
			    point desired_pos,
			    point current_dir,
			    double rad);

  void from_to_with_G0_drag_knife(double safe_height,
				  double align_depth,
				  gprog* p,
				  point last_pos,
				  point last_orient,
				  point next_pos,
				  point next_orient);
}

#endif
