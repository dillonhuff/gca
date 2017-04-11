#include "process_planning/tool_access_depth_field.h"

#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "process_planning/major_axis_fixturing.h"
#include "synthesis/millability.h"

namespace gca {

  polygon_3 surface_boundary_polygon(const surface& s) {
    return surface_boundary_polygon(s.index_list(), s.get_parent_mesh());
  }

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

    vector<surface> non_vertical =
      select(const_surfs,
	     [n](const surface& s) {
	       return !angle_eps(normal(s), n, 90.0, 2.0);
	     });

    double field_resolution = 0.05;
    double min_value = min_in_dir(m, point(0, 0, 1));
    depth_field part_field =
      build_from_stl(m.bounding_box(), m, min_value, field_resolution);
    // TODO: Adjust this to use swept volume of tool
    depth_field t_field =
      min_tool_height_field(t, part_field);

    vtk_debug_depth_field(part_field);
    vtk_debug_depth_field(t_field);

    vector<surface> accessable;
    for (auto& nv : non_vertical) {
      auto bound_poly = surface_boundary_polygon(nv);
      auto bound_poly_2 = to_boost_poly_2(bound_poly);

      bool contained = true;
      for (int i = 0; i < t_field.num_x_elems; i++) {

	double x = t_field.x_coord(i);

	for (int j = 0; j < t_field.num_y_elems; j++) {

	  double y = t_field.y_coord(j);

	  auto pt = boost::geometry::model::d2::point_xy<double>(x, y);

	  if (boost::geometry::within(pt, bound_poly_2)) {

	    double pf_height =
	      part_field.column_height(i, j);
	    double tf_height =
	      t_field.column_height(i, j);

	    if (pf_height < tf_height) {

	      cout << "NOT CONTAINED" << endl;
	      vtk_debug_highlight_inds(nv);

	      contained = false;
	      break;
	    }
	  }
	  
	}

	if (contained == false) {
	  break;
	}
      }

      if (contained) {
	accessable.push_back(nv);
      }
    }

    cout << "# accessable surfaces = " << accessable.size() << endl;
    
    return accessable;
  }

}

