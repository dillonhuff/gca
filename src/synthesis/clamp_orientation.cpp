#include "geometry/vtk_debug.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/millability.h"

namespace gca {

  surface
  contact_surface(const plane p, const triangular_mesh& part) {
    vector<index_t> inds_along;
    for (auto i : part.face_indexes()) {
      triangle t = part.face_triangle(i);
      if (within_eps(t.normal, p.normal(), 0.001)) {
	if (within_eps(p.normal().dot(p.pt() - t.v1), 0, 0.001)) {
	  inds_along.push_back(i);
	}
      }
    }
    return surface(&part, inds_along);
  }

  double clamp_orientation::contact_area(const triangular_mesh& m) const {
    surface l = contact_surface(left_plane(), m);
    surface r = contact_surface(left_plane(), m);
    surface b = contact_surface(right_plane(), m);
    return l.surface_area() + r.surface_area() + b.surface_area();
  }

  homogeneous_transform mating_transform(const triangular_mesh& m,
					 const clamp_orientation& orient,
					 const vice& v) {
    plane vice_base = v.base_plane();
    plane vice_top_jaw = v.top_jaw_plane();
    plane vice_right_bound = v.right_bound_plane();
    
    plane mesh_base = orient.base_plane();
    plane mesh_left = orient.left_plane();

    point free_axis = cross(mesh_base.normal(), mesh_left.normal());

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
  oriented_part_mesh(const triangular_mesh& m,
		     const clamp_orientation& orient,
		     const vice v) {
    homogeneous_transform t = mating_transform(m, orient, v);
    return apply(t, m);
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
  all_clamp_orientations_with_top_normal(const std::vector<surface>& surfaces,
					 const point n) {
    vector<clamp_orientation> orients;
    for (auto& next_bottom : surfaces) {

      if (angle_eps(normal(next_bottom), n, 180, 1.0)) {
	for (auto& next_left : surfaces) {
	  for (auto& next_right : surfaces) {

	    if (parallel_flat_surfaces(&next_right, &next_left)) {

	      if (orthogonal_flat_surfaces(&next_bottom, &next_left) &&
		  orthogonal_flat_surfaces(&next_bottom, &next_right)) {
		orients.push_back(clamp_orientation(&next_left,
						    &next_right,
						    &next_bottom));
	      }

	    }
	  }
	}
      }
    }
    return orients;
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
    		auto left_pt = orient.left_plane_point();
    		auto right_pt = orient.right_plane_point();
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

    return orients;
  }

  std::vector<clamp_orientation>
  all_stable_orientations_with_top_normal(const std::vector<surface>& surfaces,
					  const vice& v,
					  const point n) {
    vector<clamp_orientation> orients =
      all_clamp_orientations_with_top_normal(surfaces, n);

    filter_stub_orientations(orients, v);

    unique_by_top_normal(orients);

    return orients;
  }
  
  bool point_above_vice(const point p,
			const clamp_orientation& orient,
			const vice& v) {
    point vice_dir = orient.top_normal();
    point vice_jaw_top = orient.bottom_plane_point() + (v.jaw_height() * vice_dir);
    return signed_distance_along(p, vice_dir) >
      signed_distance_along(vice_jaw_top, vice_dir);
  }

  std::vector<index_t> vertices_along(const triangular_mesh& part,
				      const plane p) {
    vector<index_t> inds_along;
    for (auto i : part.face_indexes()) {
      triangle t = part.face_triangle(i);
      if (within_eps(t.normal, p.normal(), 0.001)) {
	if (within_eps(p.normal().dot(p.pt() - t.v1), 0, 0.001)) {
	  triangle_t ti = part.triangle_vertices(i);
	  inds_along.push_back(ti.v[0]);
	  inds_along.push_back(ti.v[1]);
	  inds_along.push_back(ti.v[2]);
	}
      }
    }
    return inds_along;
  }

  std::vector<index_t> vertexes_touching_fixture(const triangular_mesh& part,
						 const clamp_orientation& orient,
						 const vice& v) {
    vector<index_t> vert_inds;
    concat(vert_inds, vertices_along(part, orient.left_plane()));
    concat(vert_inds, vertices_along(part, orient.right_plane()));

    if (!v.has_parallel_plate()) {
      concat(vert_inds, vertices_along(part, orient.base_plane()));
    }
    sort(begin(vert_inds), end(vert_inds));
    delete_if(vert_inds,
	      [v, orient, part](const index_t i)
	      { return point_above_vice(part.vertex(i), orient, v); });
    return vert_inds;
  }

  std::vector<unsigned>
  surfaces_millable_from(const clamp_orientation& orient,
			 const std::vector<surface*>& surfaces_left,
			 const vice& v) {
    assert(surfaces_left.size() > 0);
    const triangular_mesh& part = surfaces_left.front()->get_parent_mesh();

    std::vector<index_t> millable =
      millable_faces(orient.top_normal(), part);
    vector<index_t> verts_touching_fixture =
      vertexes_touching_fixture(part, orient, v);
    delete_if(millable,
	      [&verts_touching_fixture, part](const index_t face_ind)
	      { return any_vertex_in(part.triangle_vertices(face_ind),
				     verts_touching_fixture); });
    sort(begin(millable), end(millable));
    vector<unsigned> mill_surfaces;
    for (unsigned i = 0; i < surfaces_left.size(); i++) {
      if (surfaces_left[i]->contained_by_sorted(millable)) {
	mill_surfaces.push_back(i);
      }
    }
    return mill_surfaces;
  }

  clamp_orientation
  find_orientation_by_normal(const std::vector<clamp_orientation>& orients,
			     const point n) {
    auto r = find_if(begin(orients), end(orients),
		     [n](const clamp_orientation& s)
		     { return within_eps(angle_between(s.top_normal(), n), 0, 1.0); });
    assert(r != end(orients));
    return *r;
  }

  std::vector<clamp_orientation>
  three_orthogonal_orients(const std::vector<clamp_orientation>& orients) {
    auto orthogonal_to = [](const clamp_orientation& l,
			    const clamp_orientation& r)
      { return within_eps(l.top_normal().dot(r.top_normal()), 0, 0.001); };
    
    return take_basis(orients, orthogonal_to, 3);
  }

  clamp_orientation
  largest_upward_orientation(const std::vector<surface>& surfs,
			     const vice& parallel,
			     const point n) {
    vector<clamp_orientation> orients =
      all_viable_clamp_orientations(surfs, parallel);


    if (!(orients.size() > 0)) {
      vtk_debug_highlight_inds(surfs);
      DBG_ASSERT(orients.size() > 0);
    }

    vector<clamp_orientation> top_orients =
      select(orients, [n](const clamp_orientation& s)
	     { return within_eps(angle_between(s.top_normal(), n), 0, 1.0); });

    DBG_ASSERT(top_orients.size() > 0);

    const triangular_mesh& m = surfs.front().get_parent_mesh();
    sort(begin(top_orients), end(top_orients),
	 [m](const clamp_orientation& l, const clamp_orientation& r)
	 { return l.contact_area(m) > r.contact_area(m); });

    DBG_ASSERT(top_orients.size() > 0);
      
    return top_orients.front();
  }

}
