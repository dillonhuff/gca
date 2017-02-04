#ifndef GCA_REGION_H
#define GCA_REGION_H

#include <utility>

#include "simulators/mill_tool.h"

using namespace std;

namespace gca {

  class depth_field {
  protected:
    point origin;
    point x_axis, y_axis;
    float* column_heights;

  public:
    double resolution;
    double x_len, y_len;
    int num_x_elems, num_y_elems;

    depth_field(const point p_origin,
		const point p_x_axis,
		const point p_y_axis,
		double x_w,
		double y_w,
		double z_w,
		double xy_resolution) :
      origin(p_origin), x_axis(p_x_axis), y_axis(p_y_axis),
      resolution(xy_resolution), x_len(x_w), y_len(y_w),
      num_x_elems(x_w/static_cast<double>(xy_resolution)),
      num_y_elems(y_w/static_cast<double>(xy_resolution)) {

      column_heights = static_cast<float*>(malloc(sizeof(float)*num_x_elems*num_y_elems));
      for (int i = 0; i < num_x_elems; i++) {
	for (int j = 0; j < num_y_elems; j++) {
	  set_column_height(i, j, 0);
	}
      }
    }

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
    
    ~depth_field() {
      delete column_heights;
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

  };

  class region {
  public:
    depth_field r;
    double total_volume_removed;
    double machine_x_offset, machine_y_offset, machine_z_offset;

    region(double x_w,
	   double y_w,
	   double z_w,
	   double xy_resolution) :
      r(point(0, 0, 0), point(1, 0, 0), point(0, 1, 0),
	x_w, y_w, z_w, xy_resolution),
      total_volume_removed(0),
      machine_x_offset(0), machine_y_offset(0), machine_z_offset(0) {}
    
    region(const point origin,
	   const point x_axis,
	   const point y_axis,
	   double x_w,
	   double y_w,
	   double z_w,
	   double xy_resolution) :
      r(origin, x_axis, y_axis, x_w, y_w, z_w, xy_resolution),
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
      int first_x = static_cast<int>(t.x_min(p) / r.resolution);
      int last_x = static_cast<int>(t.x_max(p) / r.resolution) + 1;
      int first_y = static_cast<int>(t.y_min(p) / r.resolution);
      int last_y = static_cast<int>(t.y_max(p) / r.resolution) + 1;

      for (int i = first_x; i < last_x; i++) {
	for (int j = first_y; j < last_y; j++) {
	  if (t.contains(p, r.resolution, i, j)) {
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
      int first_x = static_cast<int>(t.x_min(p) / r.resolution);
      int last_x = static_cast<int>(t.x_max(p) / r.resolution);
      int first_y = static_cast<int>(t.y_min(p) / r.resolution);
      int last_y = static_cast<int>(t.y_max(p) / r.resolution);

      for (int i = first_x; i < last_x; i++) {
	for (int j = first_y; j < last_y; j++) {
	  if (t.contains(p, r.resolution, i, j) && !r.legal_column(i, j)) {
	    return false;
	  }
	}
      }

      return true;
    }

  };

}

#endif
