#ifndef GCA_VICE_H
#define GCA_VICE_H

#include "geometry/box.h"
#include "geometry/point.h"

namespace gca {

  typedef double plate_height;

  class vice {
  private:
    point pos;
    double x_length, y_length, base_height, top_height, clamp_width, max_jaw_width, protective_base_plate_height;
    
  public:
    vice(point p_pos,
	 double p_x_length,
	 double p_y_length,
	 double p_base_height,
	 double p_top_height,
	 double p_clamp_width,
	 double p_max_jaw_width) :
      pos(p_pos),
      x_length(p_x_length),
      y_length(p_y_length),
      base_height(p_base_height),
      top_height(p_top_height),
      clamp_width(p_clamp_width),
      max_jaw_width(p_max_jaw_width),
      protective_base_plate_height(0.0) {}

    vice(point p_pos,
	 double p_x_length,
	 double p_y_length,
	 double p_base_height,
	 double p_top_height,
	 double p_clamp_width,
	 double p_max_jaw_width,
	 double p_protective_base_plate_height) :
      pos(p_pos),
      x_length(p_x_length),
      y_length(p_y_length),
      base_height(p_base_height),
      top_height(p_top_height),
      clamp_width(p_clamp_width),
      max_jaw_width(p_max_jaw_width),
      protective_base_plate_height(p_protective_base_plate_height) {}
    
    vice(const vice& v,
	 double p_protective_base_plate_height) :
      pos(v.pos),
      x_length(v.x_length),
      y_length(v.y_length),
      base_height(v.base_height),
      top_height(v.top_height),
      clamp_width(v.clamp_width),
      max_jaw_width(v.max_jaw_width),
      protective_base_plate_height(p_protective_base_plate_height) {}

    inline bool has_protective_base_plate() const
    { return protective_base_plate_height != 0.0; }
    
    inline double x_min() const { return pos.x; }
    inline double y_min() const { return pos.y; }
    inline double z_min() const { return pos.z; }

    inline double x_max() const { return pos.x + x_length; }
    inline double y_max() const { return pos.y + y_length; }

    inline double fixed_clamp_y() const { return y_max() - clamp_width; }
    inline double clamp_y_length() const { return clamp_width; }

    inline double jaw_height() const { return top_height - (base_height + protective_base_plate_height); }

    inline double base_z() const { return pos.z + base_height + protective_base_plate_height; }
    inline double top_z() const { return pos.z + top_height; }

    inline point position() const { return pos; }

    inline double maximum_jaw_width() const { return max_jaw_width; }

  };

  vice emco_vice(const point loc);

  vice current_setup();

  box main_box(const vice v);

  box upper_clamp_box(const vice v);

  box lower_clamp_box(const vice v);

}

#endif
