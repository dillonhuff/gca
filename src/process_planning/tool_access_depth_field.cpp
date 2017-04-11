#include "process_planning/tool_access_depth_field.h"

#include "geometry/vtk_debug.h"
#include "process_planning/major_axis_fixturing.h"
#include "synthesis/millability.h"

namespace gca {

  std::vector<surface> accessable_surfaces(const triangular_mesh& m,
					   const tool& t) {
    point n(0, 0, 1);
    auto inds = millable_faces(n, m);
    sort(begin(inds), end(inds));

    auto regions = const_orientation_regions(m);
    vector<surface> const_surfs = inds_to_surfaces(regions, m);

    delete_if(const_surfs,
	      [inds](const surface& s) {
		return !s.contained_by_sorted(inds);
	      });

    double field_resolution = 0.1;
    double min_value = min_in_dir(m, point(0, 0, 1));
    depth_field part_field =
      build_from_stl(m.bounding_box(), m, min_value, field_resolution);
    depth_field t_field =
      min_tool_height_field(t, part_field);

    vtk_debug_depth_field(part_field);
    vtk_debug_depth_field(t_field);
    
    return const_surfs;
  }

}

