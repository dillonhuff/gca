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

    inline double x_min() const { return pos.x; }
    inline double y_min() const { return pos.y; }
    inline double z_min() const { return pos.z; }

    inline double x_max() const { return pos.x + x_length; }
    inline double y_max() const { return pos.y + y_length; }

    inline double fixed_clamp_y() const { return y_max() - clamp_width; }

    inline double jaw_height() const { return top_height - base_height; }

    inline double base_z() const { return pos.z + base_height; }
    inline double top_z() const { return pos.z + top_height; }

    inline point position() const { return pos; }

  };

  vice emco_vice(const point loc);

  vice current_setup();  
}

#endif
