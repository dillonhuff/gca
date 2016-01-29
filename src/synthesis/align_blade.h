#ifndef GCA_ALIGN_BLADE_H
#define GCA_ALIGN_BLADE_H

#include "geometry/point.h"

namespace gca {

  void align_coords(point desired_dir,
		    point desired_pos,
		    point current_dir,
		    double rad,
		    point& c_pos,
		    point& center_off);
  
}

#endif
