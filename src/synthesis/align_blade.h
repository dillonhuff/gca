#ifndef GCA_ALIGN_BLADE_H
#define GCA_ALIGN_BLADE_H

#include "geometry/point.h"
#include "gcode/circular_arc.h"

namespace gca {

  circular_arc align_coords(point desired_dir,
			    point desired_pos,
			    point current_dir,
			    double rad);

  vector<cut*> from_to_with_G0_drag_knife(double safe_height,
					  double align_depth,
					  point last_pos,
					  point last_orient,
					  point next_pos,
					  point next_orient);
}

#endif
