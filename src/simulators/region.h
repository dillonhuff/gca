#ifndef GCA_REGION_H
#define GCA_REGION_H

#include <utility>

#include "simulators/mill_tool.h"

using namespace std;

namespace gca {

  // template<typename Num>
  // class depth_field {
  // public:
  // };

  class region {
  public:
    double resolution;
    //double height;
    double x_len, y_len;
    int num_x_elems, num_y_elems;
    double total_volume_removed;
    float* column_heights;
    double machine_x_offset, machine_y_offset, machine_z_offset;
    
  region(double x_w, double y_w, double z_w, double xy_resolution) :
    resolution(xy_resolution), x_len(x_w), y_len(y_w),
    //resolution(xy_resolution), height(z_w), x_len(x_w), y_len(y_w),
      num_x_elems(x_w/static_cast<double>(xy_resolution)),
      num_y_elems(y_w/static_cast<double>(xy_resolution)),
      total_volume_removed(0),
    machine_x_offset(0), machine_y_offset(0), machine_z_offset(0) {
      column_heights = static_cast<float*>(malloc(sizeof(float)*num_x_elems*num_y_elems));
      for (int i = 0; i < num_x_elems; i++) {
	for (int j = 0; j < num_y_elems; j++) {
	  set_column_height(i, j, 0);
	}
      }
    }

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

    void set_height(double x_s, double x_e,
		    double y_s, double y_e,
		    double h) {
      int first_x = static_cast<int>(x_s / resolution);
      int last_x = static_cast<int>(x_e / resolution);
      int first_y = static_cast<int>(y_s / resolution);
      int last_y = static_cast<int>(y_e / resolution);
      for (int i = first_x; i < last_x; i++) {
	for (int j = first_y; j < last_y; j++) {
	  set_column_height(i, j, h);
	}
      }      
    }
    
    ~region() {
      delete column_heights;
    }

    double volume_removed() {
      return total_volume_removed;
    }

    inline float column_height(int i, int j) const {
      return *(column_heights + i*num_y_elems + j);
    }

    inline void set_column_height(int i, int j, float f) {
      *(column_heights + i*num_y_elems + j) = f;
    }

    inline bool legal_column(int i, int j) const {
      return (0 <= i && i < num_x_elems) && (0 <= j && j < num_y_elems);
    }

    inline bool in_region(point p, const mill_tool& t) const {
      int first_x = static_cast<int>(t.x_min(p) / resolution);
      int last_x = static_cast<int>(t.x_max(p) / resolution);
      int first_y = static_cast<int>(t.y_min(p) / resolution);
      int last_y = static_cast<int>(t.y_max(p) / resolution);

      for (int i = first_x; i < last_x; i++) {
	for (int j = first_y; j < last_y; j++) {
	  if (t.contains(p, resolution, i, j) && !legal_column(i, j)) {
	    return false;
	  }
	}
      }

      return true;
    }

    double update(point p, const mill_tool& t) {
      double volume_removed = 0.0;
      int first_x = static_cast<int>(t.x_min(p) / resolution);
      int last_x = static_cast<int>(t.x_max(p) / resolution) + 1;
      int first_y = static_cast<int>(t.y_min(p) / resolution);
      int last_y = static_cast<int>(t.y_max(p) / resolution) + 1;

      for (int i = first_x; i < last_x; i++) {
	for (int j = first_y; j < last_y; j++) {
	  if (t.contains(p, resolution, i, j)) {
	    double h = static_cast<double>(column_height(i, j));
	    if (h > p.z) {
	      double z_diff = h - p.z;
	      set_column_height(i, j, p.z);
	      volume_removed += resolution*resolution*z_diff;
	    }
	  }
	}
      }

      total_volume_removed += volume_removed;
      return volume_removed;
    }
  };

}

#endif
