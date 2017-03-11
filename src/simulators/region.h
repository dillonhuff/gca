#ifndef GCA_REGION_H
#define GCA_REGION_H

#include <utility>

#include "geometry/depth_field.h"
#include "simulators/mill_tool.h"

using namespace std;

namespace gca {

  class region {
  public:
    depth_field r;
    double total_volume_removed;
    double machine_x_offset, machine_y_offset, machine_z_offset;

    region(const depth_field& p_r) :
      r(p_r),
      total_volume_removed(0),
      machine_x_offset(0), machine_y_offset(0), machine_z_offset(0) {}
    
    region(double x_w,
	   double y_w,
	   double z_w,
	   double xy_resolution) :
      r(point(0, 0, 0),
	x_w, y_w, xy_resolution),
      total_volume_removed(0),
      machine_x_offset(0), machine_y_offset(0), machine_z_offset(0) {}
    
    region(const point origin,
	   double x_w,
	   double y_w,
	   double z_w,
	   double xy_resolution) :
      r(origin, x_w, y_w, xy_resolution),
      total_volume_removed(0),
      machine_x_offset(0), machine_y_offset(0), machine_z_offset(0) {}

    // TODO: Generalize to allow for user specification of
    // machine and region coordinates
    point machine_coords_to_region_coords(point p) const {
      return point(p.x + machine_x_offset,
		   p.y + machine_y_offset,
		   p.z + machine_z_offset);
    }

    point region_coords_to_machine_coords(const point p) const {
      return point(p.x - machine_x_offset,
		   p.y - machine_y_offset,
		   p.z - machine_z_offset);
    }
    
    inline void set_machine_x_offset(double x_off) { machine_x_offset = x_off; }
    inline void set_machine_y_offset(double y_off) { machine_y_offset = y_off; }
    inline void set_machine_z_offset(double z_off) { machine_z_offset = z_off; }
    
    double volume_removed() {
      return total_volume_removed;
    }

    double update(point p, const mill_tool& t) {
      double volume_removed = 0.0;

      int first_x = r.x_index(t.x_min(p));
      int last_x = r.x_index(t.x_max(p)) + 1;
      int first_y = r.y_index(t.y_min(p));
      int last_y = r.y_index(t.y_max(p)) + 1;
      
      for (int i = first_x; i < last_x; i++) {
	for (int j = first_y; j < last_y; j++) {
	  if (t.contains(p, r.get_origin(), r.resolution, i, j) &&
	      r.legal_column(i, j)) {
	    double h = static_cast<double>(r.column_height(i, j));
	    if (h > p.z) {
	      double z_diff = h - p.z;
	      r.set_column_height(i, j, p.z);
	      volume_removed += r.resolution*r.resolution*z_diff;
	    }
	  }
	}
      }

      total_volume_removed += volume_removed;
      return volume_removed;
    }

    inline bool in_region(point p, const mill_tool& t) const {

      int first_x = r.x_index(t.x_min(p));
      int last_x = r.x_index(t.x_max(p));
      int first_y = r.y_index(t.y_min(p));
      int last_y = r.y_index(t.y_max(p));

      for (int i = first_x; i < last_x; i++) {
	for (int j = first_y; j < last_y; j++) {
	  if (t.contains(p, r.get_origin(), r.resolution, i, j) && !r.legal_column(i, j)) {
	    return false;
	  }
	}
      }

      return true;
    }

  };

}

#endif
