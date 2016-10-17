#include "synthesis/vice.h"

namespace gca {

  vice::vice(point p_pos,
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
    parallel_plate_height(0.0) {

    DBG_ASSERT(y_length > max_jaw_width + 2*clamp_width);
  }

  vice::vice(point p_pos,
	     double p_x_length,
	     double p_y_length,
	     double p_base_height,
	     double p_top_height,
	     double p_clamp_width,
	     double p_max_jaw_width,
	     double p_parallel_plate_height) :
    pos(p_pos),
    x_length(p_x_length),
    y_length(p_y_length),
    base_height(p_base_height),
    top_height(p_top_height),
    clamp_width(p_clamp_width),
    max_jaw_width(p_max_jaw_width),
    parallel_plate_height(p_parallel_plate_height) {

    DBG_ASSERT(y_length > max_jaw_width + 2*clamp_width);
  }
    
  vice::vice(const vice& v,
	     double p_parallel_plate_height) :
    pos(v.pos),
    x_length(v.x_length),
    y_length(v.y_length),
    base_height(v.base_height),
    top_height(v.top_height),
    clamp_width(v.clamp_width),
    max_jaw_width(v.max_jaw_width),
    parallel_plate_height(p_parallel_plate_height) {

    DBG_ASSERT(y_length > max_jaw_width + 2*clamp_width);
  }
  
  vice top_jaw_origin_vice(const vice& v) {
    vice origin_vice(v);
    origin_vice.set_position(point(0.0 - v.x_len(),
				   0.0 - v.y_len() + v.clamp_y_length(),
				   0.0 - v.z_len()));
    return origin_vice;
  }

  vice emco_vice(const point loc) {
    return large_jaw_vice(2.0, loc);
  }

  vice large_jaw_vice(const double jaw_width, const point loc) {
    return vice(loc, 2.5, 5.5, 1.1, 1.87, 1.3, jaw_width);
  }

  vice custom_jaw_vice(const double jaw_width,
		       const double clamp_width,
		       const double y_length,
		       const point loc) {
    return vice(loc, 2.5, y_length, 1.1, 1.87, clamp_width, jaw_width);
  }

  vice current_setup() {
    return emco_vice(point(-0.8, -4.4, -6.3));
  }

  box main_box(const vice v) {
    cout << "vice z min = " << v.z_min() << endl;
    return box(v.x_min(), v.x_max(), v.y_min(), v.y_max(), v.z_min(), v.base_z());
  }

  box upper_clamp_box(const vice v) {
    return box(v.x_min(), v.x_max(),
	       v.fixed_clamp_y(), v.y_max(),
	       v.base_z(), v.top_z());
  }

  box lower_clamp_box(const vice v) {
    return box(v.x_min(), v.x_max(),
	       v.y_min(), v.y_min() + v.clamp_y_length(),
	       v.base_z(), v.top_z());
  }

  vice shift(const point s, const vice& v) {
    vice shifted = v;

    shifted.set_position(v.position() + s);

    return shifted;
  }
}
