#ifndef GCA_VICE_H
#define GCA_VICE_H

#include "geometry/point.h"

namespace gca {

  class vice {
  private:
    point pos;
    double x_length, y_length, base_height, top_height, clamp_width;
    
  public:
    vice(point p_pos,
	 double p_x_length,
	 double p_y_length,
	 double p_base_height,
	 double p_top_height,
	 double p_clamp_width) :
      pos(p_pos),
      x_length(p_x_length),
      y_length(p_y_length),
      base_height(p_base_height),
      top_height(p_top_height),
      clamp_width(p_clamp_width) {}

    // TODO: Fill in with actual vice position parameters
    inline double x_max() const { return pos.x + x_length; }
    inline double y_max() const { return pos.x + y_length; }
    inline double fixed_clamp_y() const { return pos.y + clamp_width; }
    inline double base_z() const { return pos.z + base_height; }
    inline double top_z() const { return pos.z + top_height; }

  };

  vice emco_vice(const point loc);

}

#endif
