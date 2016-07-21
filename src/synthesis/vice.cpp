#include "synthesis/vice.h"

namespace gca {

  vice emco_vice(const point loc) {
    return large_jaw_vice(2.0, loc);
  }

  vice large_jaw_vice(const double jaw_width, const point loc) {
    return vice(loc, 2.5, 5.5, 1.1, 1.87, 1.3, jaw_width);
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

}
