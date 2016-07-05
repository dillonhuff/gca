#include "utils/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/sim_mill.h"
#include "gcode/circular_arc.h"
#include "gcode/linear_cut.h"

namespace gca {

  region bounding_region(double tool_diameter, box b, double material_height) {
    double x_len = b.x_max - b.x_min + 5*tool_diameter;
    double y_len = b.y_max - b.y_min + 5*tool_diameter;
    double z_len = b.z_max - b.z_min;
    // if (!(b.z_max > material_height)) {
    //   cout << "ERROR" << endl;
    //   cout << "z_max = " << b.z_max << endl;
    //   cout << "material height = " << material_height << endl;
    //   assert(false);
    // }
    region r(x_len, y_len, z_len, 0.01);
    r.set_machine_x_offset(-b.x_min + 2*tool_diameter);
    r.set_machine_y_offset(-b.y_min + 2*tool_diameter);
    r.set_machine_z_offset(-b.z_min);
    // TODO: Find better way to express the safe z value in the
    // machine coordinate system
    point safe_machine_point(0, 0, material_height);
    point safe_region_point = r.machine_coords_to_region_coords(safe_machine_point);
    r.set_height(0, x_len, 0, y_len, safe_region_point.z);
    return r;
  }

  region set_up_region_conservative(const vector<vector<cut*>>& paths,
				    double tool_diameter) {
    box b = bound_paths(paths);
    double material_h = infer_material_height(paths, 0.01);
    return bounding_region(tool_diameter, b, material_h);
  }

  region set_up_region(const vector<vector<cut*>>& paths,
		       double tool_diameter) {
    box b = bound_paths(paths);
    double safe_z = infer_safe_height(paths);
    return bounding_region(tool_diameter, b, safe_z);
  }

  void check_region_bounds(const point e, const region& r, const mill_tool& t) {
    if (!r.in_region(e, t)) {
      cout << e << " goes outside of region bounds" << endl;
      cout << "Region height: " << r.height << endl;
      cout << "Region x len: " << r.x_len << endl;
      cout << "Region y len: " << r.y_len << endl;
      cout << "Machine x offset: " << r.machine_x_offset << endl;
      cout << "Machine y offset: " << r.machine_y_offset << endl;
      cout << "Machine z offset: " << r.machine_z_offset << endl;
      assert(false);
    }
  }

  double update_cut(const cut& c, region& r, const mill_tool& t) {
    double volume_removed = 0.0;
    double d = r.resolution;
    int num_points = (c.length() / d) + 1;
    for (int i = 0; i < num_points; i++) {
      double tp = static_cast<double>(i) / static_cast<double>(num_points);
      point e = r.machine_coords_to_region_coords(c.value_at(tp));
      check_region_bounds(e, r, t);
      volume_removed += r.update(e, t);
    }
    return volume_removed;
  }
  
  double simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t) {
    double volume_removed = 0.0;
    for (auto c : p) {
      volume_removed += update_cut(*c, r, t);
    }
    return volume_removed;
  }

}
