#ifndef GCA_VICE_H
#define GCA_VICE_H

#include "geometry/box.h"
#include "geometry/plane.h"
#include "geometry/point.h"

namespace gca {

  typedef double plate_height;

  class vice {
  private:
    point pos;
    double x_length, y_length, base_height, top_height, clamp_width, opening_capacity, parallel_plate_height;

    point base_normal, top_clamp_normal;

  public:
    vice(point p_pos,
    	 double p_x_length,
    	 double p_y_length,
    	 double p_base_height,
    	 double p_top_height,
    	 double p_clamp_width,
    	 double p_opening_capacity,
    	 point p_base_normal,
    	 point p_top_clamp_normal);

    vice(point p_pos,
	 double p_x_length,
	 double p_y_length,
	 double p_base_height,
	 double p_top_height,
	 double p_clamp_width,
	 double p_opening_capacity,
	 double p_parallel_plate_height,
	 point p_base_normal,
	 point p_top_clamp_normal);

    vice(const vice& v,
	 double p_parallel_plate_height);

    inline vice without_extras() const {
      return vice(*this, 0);
    }

    inline bool has_parallel_plate() const
    { return parallel_plate_height != 0.0; }

    inline double x_min() const { return pos.x; }
    inline double y_min() const { return pos.y; }
    inline double z_min() const { return pos.z; }

    inline double x_max() const { return pos.x + x_length; }
    inline double y_max() const { return pos.y + y_length; }
    inline double z_max() const { return pos.z + top_height; }

    inline double x_len() const { return x_length; }
    inline double y_len() const { return y_length; }
    inline double z_len() const { return top_height; }

    inline double fixed_clamp_y() const { return y_max() - clamp_width; }
    inline double clamp_y_length() const { return clamp_width; }
    inline double jaw_height() const
    { return top_height - (base_height + parallel_plate_height); }

    inline double plate_height() const
    { return parallel_plate_height; }

    inline double base_z() const
    { return pos.z + base_height + parallel_plate_height; }
    inline double top_z() const { return pos.z + top_height; }

    inline point position() const { return pos; }

    inline void set_position(const point new_pos) { pos = new_pos; }

    inline double max_opening_capacity() const { return opening_capacity; }

    inline point base_face_normal() const { return base_normal; }

    inline point top_clamping_face_normal() const { return top_clamp_normal; }

    inline point right_bound_normal() const {
      point rbn = cross(top_clamping_face_normal(), base_face_normal());
      return rbn;
    }

    inline plane base_plane() const {
      return plane(base_face_normal(),
		   point(x_max(), fixed_clamp_y(), base_z()));
    }

    inline plane top_jaw_plane() const {
      return plane(top_clamping_face_normal(),
		   point(x_max(), fixed_clamp_y(), base_z()));
    }

    inline plane right_bound_plane() const {
      return plane(right_bound_normal(),
		   point(x_max(), fixed_clamp_y(), base_z()));
    }

  };

  vice emco_vice(const point loc);
  vice large_jaw_vice(const double jaw_width, const point loc);
  vice current_setup();

  box main_box(const vice v);

  box upper_clamp_box(const vice v);

  box lower_clamp_box(const vice v);

  vice top_jaw_origin_vice(const vice& v);

  vice custom_jaw_vice(const double jaw_width,
		       const double clamp_width,
		       const double y_length,
		       const point loc);

  vice shift(const point s, const vice& v);

  vice custom_jaw_vice_with_clamp_dir(const double jaw_width,
				      const double clamp_width,
				      const double y_length,
				      const point loc,
				      const point top_clamp_face_normal);
  
}

#endif
