#include "geometry/homogeneous_transformation.h"
#include "synthesis/clamp_orientation.h"

namespace gca {

  homogeneous_transform mating_transform(const clamp_orientation& orient,
					 const vice& v) {
    plane vice_base = v.base_plane();
    plane vice_top_jaw = v.top_jaw_plane();
    plane vice_right_bound = v.right_bound_plane();
    
    plane mesh_base = orient.base_plane();
    plane mesh_left = orient.left_plane();

    point free_axis = cross(mesh_base.normal(), mesh_left.normal());
    cout << "mesh base normal = " << mesh_base.normal() << endl;
    cout << "mesh left normal = " << mesh_left.normal() << endl;
    cout << "free axis = " << free_axis << endl;
    const triangular_mesh& m = orient.get_mesh();
    plane free_plane(free_axis, max_point_in_dir(m, free_axis));

    boost::optional<homogeneous_transform> t =
      mate_planes(mesh_base, mesh_left, free_plane,
		  vice_base, vice_top_jaw, vice_right_bound);
    if (t) {
      return *t;
    }
    
    free_axis = -1* free_axis;
    free_plane = plane(free_axis, max_point_in_dir(m, free_axis));

    boost::optional<homogeneous_transform> rev =
      mate_planes(mesh_base, mesh_left, free_plane,
		  vice_base, vice_top_jaw, vice_right_bound);

    assert(rev);
    return *rev;
  }

  triangular_mesh
  oriented_part_mesh(const clamp_orientation& orient,
		     const vice v) {
    homogeneous_transform t = mating_transform(orient, v);
    return apply(t, orient.get_mesh());
  }

}
