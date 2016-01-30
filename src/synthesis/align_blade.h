#ifndef GCA_ALIGN_BLADE_H
#define GCA_ALIGN_BLADE_H

#include "geometry/point.h"

namespace gca {

  double angle_between(point u, point v);

  void align_coords(point desired_dir,
		    point desired_pos,
		    point current_dir,
		    double rad,
		    point& c_pos,
		    point& center_off);

  void from_to_with_G0_drag_knife(double safe_height,
				  double align_depth,
				  gprog* p,
				  point last_pos,
				  point last_orient,
				  point next_pos,
				  point next_orient);
}

#endif
