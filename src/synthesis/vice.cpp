#include "synthesis/vice.h"

namespace gca {

  vice::vice(point p_pos,
	     double p_x_length,
	     double p_y_length,
	     double p_base_height,
	     double p_top_height,
	     double p_clamp_width,
	     double p_opening_capacity,
	     point p_base_normal,
	     point p_top_clamp_normal) :
	     
    pos(p_pos),
    x_length(p_x_length),
    y_length(p_y_length),
    base_height(p_base_height),
    top_height(p_top_height),
    clamp_width(p_clamp_width),
    opening_capacity(p_opening_capacity),
    parallel_plate_height(0.0),
    base_normal(p_base_normal),
    top_clamp_normal(p_top_clamp_normal)
  {

    DBG_ASSERT(y_length > opening_capacity + 2*clamp_width);
  }

  vice::vice(point p_pos,
	     double p_x_length,
	     double p_y_length,
	     double p_base_height,
	     double p_top_height,
	     double p_clamp_width,
	     double p_opening_capacity,
	     double p_parallel_plate_height,
	     point p_base_normal,
	     point p_top_clamp_normal) :
    pos(p_pos),
    x_length(p_x_length),
    y_length(p_y_length),
    base_height(p_base_height),
    top_height(p_top_height),
    clamp_width(p_clamp_width),
    opening_capacity(p_opening_capacity),
    parallel_plate_height(p_parallel_plate_height),
    base_normal(p_base_normal),
    top_clamp_normal(p_top_clamp_normal) {

    DBG_ASSERT(y_length > opening_capacity + 2*clamp_width);
  }
    
  vice::vice(const vice& v,
	     double p_parallel_plate_height) :
    pos(v.pos),
    x_length(v.x_length),
    y_length(v.y_length),
    base_height(v.base_height),
    top_height(v.top_height),
    clamp_width(v.clamp_width),
    opening_capacity(v.opening_capacity),
    parallel_plate_height(p_parallel_plate_height),
    base_normal(v.base_face_normal()),
    top_clamp_normal(v.top_clamping_face_normal()) {

    DBG_ASSERT(y_length > opening_capacity + 2*clamp_width);
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
    return vice(loc, 2.5, 5.5, 1.1, 1.87, 1.3, jaw_width, point(0, 0, 1), point(0, -1, 0));
  }

  vice custom_jaw_vice(const double jaw_width,
		       const double clamp_width,
		       const double y_length,
		       const point loc) {
    return vice(loc, 2.5, y_length, 1.1, 1.87, clamp_width, jaw_width, point(0, 0, 1), point(0, -1, 0));
  }

  vice custom_jaw_vice_with_clamp_dir(const double jaw_width,
				      const double clamp_width,
				      const double y_length,
				      const point loc,
				      const point top_clamp_face_normal) {
    return vice(loc, 2.5, y_length, 1.1, 1.87, clamp_width, jaw_width, point(0, 0, 1), top_clamp_face_normal);
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
