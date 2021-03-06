#ifndef GCA_REGION_H
#define GCA_REGION_H

#include <utility>

#include "geometry/depth_field.h"
#include "simulators/mill_tool.h"

using namespace std;

namespace gca {

  struct grid_cell {
    int x_ind, y_ind;
  };

  inline bool operator==(const grid_cell l, const grid_cell r) {
    return (l.x_ind == r.x_ind) && (l.y_ind == r.y_ind);
  }

  struct grid_update {
    grid_cell cell;
    double height_diff;
  };

  struct point_update {
    point cutter_location;
    double volume_removed;
    vector<grid_update> grid_updates;
  };

  inline bool same_grid_cell(const grid_update l,
			     const grid_update r) {
    return l.cell == r.cell;
  }

  std::vector<grid_update> sum_updates(const std::vector<point_update>& updates);

  //double max_cut_depth_from_updates(const std::vector<point_update>& updates);

  // double volume_removed_in_update(const double resolution,
  // 				  const point_update& update);

  double
  volume_removed_in_updates(const double resolution,
			    const std::vector<grid_update>& sum_updates);
  
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

    point_update update_at_point(const point pt, const mill_tool& t) {
      //double volume_removed = 0.0;

      point p = machine_coords_to_region_coords(pt);

      vector<grid_update> grid_updates;

      int first_x = r.x_index(t.x_min(p));
      int last_x = r.x_index(t.x_max(p)) + 1;
      int first_y = r.y_index(t.y_min(p));
      int last_y = r.y_index(t.y_max(p)) + 1;
      
      for (int i = first_x; i < last_x; i++) {
	double bl_corner_x = r.get_origin().x + i*r.resolution;

	for (int j = first_y; j < last_y; j++) {

	  double bl_corner_y = r.get_origin().y + j*r.resolution;
	  
	  //	  if (t.contains(p, r.get_origin(), r.resolution, i, j) &&
	  if (r.legal_column(i, j)) {

	    double h = static_cast<double>(r.column_height(i, j));
	    point other_pt(bl_corner_x, bl_corner_y, h);
	    
	    if (t.contains(p, other_pt)) {
	      //double h = static_cast<double>(r.column_height(i, j));
	      //	    if (h > p.z) {

	      double tool_z = t.z_at(p, other_pt);
	      double z_diff = h - tool_z;
	      r.set_column_height(i, j, tool_z);
	      grid_updates.push_back({{i, j}, z_diff});

	      //volume_removed += r.resolution*r.resolution*z_diff;
	      //	    }
	    }
	  }
	}
      }

      // total_volume_removed += volume_removed;
      // return volume_removed;

      return point_update{p, volume_removed_in_updates(r.resolution, grid_updates), grid_updates};
    }

    double update(point p, const mill_tool& t) {
      point_update updates = update_at_point(p, t);
      
      double volume_removed = 0.0;
      for (auto& g : updates.grid_updates) {
	volume_removed += g.height_diff * r.resolution*r.resolution;
      }

      return volume_removed;
    }

  };

}

#endif
