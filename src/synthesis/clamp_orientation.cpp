#include "synthesis/clamp_orientation.h"
#include "synthesis/millability.h"

namespace gca {

  homogeneous_transform mating_transform(const clamp_orientation& orient,
					 const vice& v) {
    plane vice_base = v.base_plane();
    plane vice_top_jaw = v.top_jaw_plane();
    plane vice_right_bound = v.right_bound_plane();
    
    plane mesh_base = orient.base_plane();
    plane mesh_left = orient.left_plane();

    point free_axis = cross(mesh_base.normal(), mesh_left.normal());

    const triangular_mesh& m = orient.get_mesh();
    plane free_plane(free_axis, max_point_in_dir(m, free_axis));

    boost::optional<homogeneous_transform> t =
      mate_planes(mesh_base, mesh_left, free_plane,
		  vice_base, vice_top_jaw, vice_right_bound);
    if (t) {
      return *t;
    }
    
    free_axis = -1*free_axis;
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

  std::vector<clamp_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces,
			  const vice& v) {
    std::vector<const surface*> surfs;
    for (unsigned i = 0; i < surfaces.size(); i++) {
      const surface* s = &(surfaces[i]);
      surfs.push_back(s);
    }
    return all_stable_orientations(surfs, v);
  }  

  std::vector<clamp_orientation>
  all_viable_clamp_orientations(const std::vector<const surface*>& surfaces,
				const vice& v) {
    vector<clamp_orientation> orients =
      all_clamp_orientations(surfaces);

    assert(orients.size() > 0);

    filter_stub_orientations(orients, v);

    assert(orients.size() > 0);
    
    return orients;
  }
  
  std::vector<clamp_orientation>
  all_viable_clamp_orientations(const std::vector<surface>& surfaces,
				const vice& v) {
    std::vector<const surface*> surfs;
    for (unsigned i = 0; i < surfaces.size(); i++) {
      const surface* s = &(surfaces[i]);
      surfs.push_back(s);
    }
    return all_viable_clamp_orientations(surfs, v);
  }  
  
  std::vector<clamp_orientation>
  all_clamp_orientations(const std::vector<const surface*>& surfaces) {
    vector<clamp_orientation> orients;
    for (unsigned j = 0; j < surfaces.size(); j++) {
      const surface* next_left = surfaces[j];
      for (unsigned k = 0; k < surfaces.size(); k++) {
	const surface* next_right = surfaces[k];
	if (parallel_flat_surfaces(next_right, next_left)) {
	  for (unsigned l = 0; l < surfaces.size(); l++) {
	    const surface* next_bottom = surfaces[l];
	    if (orthogonal_flat_surfaces(next_bottom, next_left) &&
		orthogonal_flat_surfaces(next_bottom, next_right)) {
	      orients.push_back(clamp_orientation(next_left,
						  next_right,
						  next_bottom));
    	    }
    	  }
    	}
      }
    }
    return orients;
  }

  void filter_stub_orientations(std::vector<clamp_orientation>& orients,
				const vice& v) {
    delete_if(orients,
    	      [v](const clamp_orientation& orient)
    	      {
    		auto part = orient.get_left().get_parent_mesh();
    		auto left_pt = part.face_triangle(orient.get_left().front()).v1;
    		auto right_pt = part.face_triangle(orient.get_right().front()).v1;
    		return abs(signed_distance_along(left_pt - right_pt, orient.left_normal()))
    		  > v.maximum_jaw_width();
    	      });

  }

  // TODO: Consider left and right normals as well, this filtering
  // criterion is too aggressive
  void unique_by_top_normal(std::vector<clamp_orientation>& orients) {
    sort(begin(orients), end(orients),
    	 [](const clamp_orientation& l, const clamp_orientation& r)
    	 { return l.top_normal().x < r.top_normal().x; });
    sort(begin(orients), end(orients),
    	 [](const clamp_orientation& l, const clamp_orientation& r)
    	 { return l.top_normal().y < r.top_normal().y; });
    sort(begin(orients), end(orients),
    	 [](const clamp_orientation& l, const clamp_orientation& r)
    	 { return l.top_normal().z < r.top_normal().z; });

    auto it = unique(begin(orients), end(orients),
    		     [](const clamp_orientation& l, const clamp_orientation& r) {
    		       return within_eps(l.top_normal(), r.top_normal(), 0.0001);
    		     });

    orients.resize(std::distance(begin(orients), it));
  }

  std::vector<clamp_orientation>
  all_stable_orientations(const std::vector<const surface*>& surfaces,
			  const vice& v) {
    vector<clamp_orientation> orients =
      all_clamp_orientations(surfaces);

    filter_stub_orientations(orients, v);

    unique_by_top_normal(orients);

    assert(orients.size() > 0);
    
    return orients;
  }

  bool point_above_vice(const index_t i,
			const clamp_orientation& orient,
			const vice& v) {
    const triangular_mesh& part = orient.get_mesh();
    point p = part.vertex(i);
    point vice_dir = orient.top_normal();
    point vice_jaw_top = orient.bottom_plane_point() + (v.jaw_height() * vice_dir);
    return signed_distance_along(p, vice_dir) >
      signed_distance_along(vice_jaw_top, vice_dir);
  }

  std::vector<index_t> vertexes_touching_fixture(const clamp_orientation& orient,
						 const vice& v) {
    vector<index_t> vert_inds;
    concat(vert_inds, surface_vertexes(orient.get_left()));
    concat(vert_inds, surface_vertexes(orient.get_right()));
    if (!v.has_protective_base_plate()) {
      concat(vert_inds, surface_vertexes(orient.get_bottom()));
    }
    sort(begin(vert_inds), end(vert_inds));
    delete_if(vert_inds,
	      [v, orient](const index_t i)
	      { return point_above_vice(i, orient, v); });
    return vert_inds;
  }

  std::vector<unsigned>
  surfaces_millable_from(const clamp_orientation& orient,
			 const std::vector<surface>& surfaces_left,
			 const vice& v) {
    const triangular_mesh& part = orient.get_mesh();
    std::vector<index_t> millable =
      millable_faces(orient.top_normal(), part);
    vector<index_t> verts_touching_fixture =
      vertexes_touching_fixture(orient, v);
    delete_if(millable,
	      [&verts_touching_fixture, part](const index_t face_ind)
	      { return any_vertex_in(part.triangle_vertices(face_ind),
				     verts_touching_fixture); });
    sort(begin(millable), end(millable));
    vector<unsigned> mill_surfaces;
    for (unsigned i = 0; i < surfaces_left.size(); i++) {
      if (surfaces_left[i].contained_by_sorted(millable)) {
	mill_surfaces.push_back(i);
      }
    }
    return mill_surfaces;
  }

}
