#include <cmath>

#include "synthesis/align_blade.h"

namespace gca {

  double angle_between(point u, point v) {
    double rads = acos((u.dot(v)) / (u.len() * v.len()));
    return (180.0/M_PI)*rads;
  }

  void align_coords(point desired_dir,
		    point desired_pos,
		    point current_dir,
		    double rad,
		    point& c_pos,
		    point& center_off) {
    double theta = -1.0 * angle_between(current_dir, desired_dir);
    cout << "theta = " << theta << endl;
    point s = desired_pos - (rad * desired_pos.normalize());
    center_off = point(0, 1, 0);
    point c_pos_prime = (desired_pos - s).rotate_z(theta);
    c_pos = s + c_pos_prime;
    center_off = -1.0 * (c_pos - s);
  }

  
}
