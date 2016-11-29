#include "geometry/extrusion.h"
#include "geometry/polygon_3.h"
#include "geometry/vtk_debug.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/millability.h"

namespace gca {

  surface
  contact_surface(const plane p, const triangular_mesh& part) {
    vector<index_t> inds_along;
    for (auto i : part.face_indexes()) {
      triangle t = part.face_triangle(i);

      if (angle_eps(t.normal, p.normal(), 0.0, 0.001)) {
	if (within_eps(p.normal().dot(p.pt() - t.v1), 0.0, 0.001)) {

	  inds_along.push_back(i);
	}
      }
    }
    return surface(&part, inds_along);
  }

  double clamp_orientation::contact_area(const triangular_mesh& m) const {
    surface l = contact_surface(left_plane(), m);
    surface r = contact_surface(right_plane(), m);
    surface b = contact_surface(base_plane(), m);
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

    DBG_ASSERT(rev);
    return *rev;
  }

  homogeneous_transform balanced_mating_transform(const triangular_mesh& m,
						  const clamp_orientation& orient,
						  const vice& v) {
    plane vice_base = v.base_plane();
    plane vice_top_jaw = v.top_jaw_plane();
    plane vice_right_bound = v.right_bound_plane();
    
    plane mesh_base = orient.base_plane();
    plane mesh_left = orient.left_plane();

    // TODO: Check that the coordinate system is right handed?
    point free_axis = cross(mesh_base.normal(), mesh_left.normal());

    // TODO: Eventually filter the points being chosen by whether
    // they will lie below the plane of the vice
    point min_pt = min_point_in_dir(m, free_axis);
    point max_pt = max_point_in_dir(m, free_axis);

    double min_dist = min_in_dir(m, free_axis);
    double max_dist = max_in_dir(m, free_axis);
    double v_x = v.x_len();

    point balanced_pt =
      min_pt + v_x*free_axis - ((v_x - (max_dist - min_dist)) / 2.0)*free_axis;

    plane free_plane(free_axis, balanced_pt);

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

    DBG_ASSERT(rev);
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

  Nef_polyhedron clip_nef(const Nef_polyhedron& part_nef,
			  const plane vice_top_plane) {
    point n = -1*vice_top_plane.normal();

    auto part = nef_to_single_trimesh(part_nef);
    polygon_3 hull = convex_hull_2D(part.vertex_list(),
				    n,
				    0.0);

    double big = 2000.0;

    polygon_3 p = project_onto(vice_top_plane, hull);
    polygon_3 plane_approx = dilate(p, big);

    triangular_mesh m = extrude(plane_approx, big*n);


    // cout << "To be clipped" << endl;
    // vtk_debug_mesh(part);

    // cout << "To clip with" << endl;
    // vtk_debug_mesh(m);

    // cout << "Clipper and clippee" << endl;
    // vtk_debug_meshes({part, m});
    
    auto clip_nef = trimesh_to_nef_polyhedron(m);
    auto cut_parts_nef = part_nef - clip_nef;

    //    DBG_ASSERT(cut_parts.size() == 1);

    return cut_parts_nef;
  }

  std::vector<clamp_orientation>
  all_stable_orientations_box(const Nef_polyhedron& part_nef,
			      const vice& v,
			      const point n) {

    auto part = nef_to_single_trimesh(part_nef);
    
    point vice_pl_pt = min_point_in_dir(part, n) + v.jaw_height()*n;
    plane vice_top_plane(-1*n, vice_pl_pt);

    Nef_polyhedron cut_parts_nef = clip_nef(part_nef, vice_top_plane);
    auto cut_part = nef_to_single_merged_trimesh(cut_parts_nef); //cut_parts.front();

    // cout << "Result of clipping" << endl;
    // vector<triangular_mesh> subs{m};
    // auto cut_parts = boolean_difference(part, subs);
    // vtk_debug_mesh(cut_part);

    vector<surface> cregions = outer_surfaces(cut_part);
    sort(begin(cregions), end(cregions),
	 [](const surface& l, const surface& r)
	 { return l.surface_area() < r.surface_area(); });
    reverse(begin(cregions), end(cregions));

    cout << "# of const orientation regions = " << cregions.size() << endl;
    // for (auto cregion : cregions) {
    //   vtk_debug_highlight_inds(cregion);
    // }

    std::vector<clamp_orientation> orients =
      all_stable_orientations_with_top_normal(cregions, v, n);

    cout << "# of orients for cregions = " << orients.size() << endl;

    return orients;
    
  }

  std::vector<std::pair<clamp_orientation, homogeneous_transform>>
  all_stable_orientations_with_side_transforms(const Nef_polyhedron& part_nef,
					       const vice& v,
					       const point n) {
    auto part = nef_to_single_trimesh(part_nef);
    polygon_3 hull = convex_hull_2D(part.vertex_list(), n, 0.0);

    point vice_pl_pt = min_point_in_dir(part, n) + v.jaw_height()*n;
    plane vice_top_plane(-1*n, vice_pl_pt);

    double big = 2000.0;

    polygon_3 p = project_onto(vice_top_plane, hull);
    polygon_3 plane_approx = dilate(p, big);

    triangular_mesh m = extrude(plane_approx, big*n);

    // cout << "To be clipped" << endl;
    // vtk_debug_mesh(part);

    // cout << "To clip with" << endl;
    // vtk_debug_mesh(m);

    // cout << "Clipper and clippee" << endl;
    // vtk_debug_meshes({part, m});

    auto clip_nef = trimesh_to_nef_polyhedron(m);
    auto cut_parts_nef = part_nef - clip_nef;

    //    DBG_ASSERT(cut_parts.size() == 1);

    auto cut_part = nef_to_single_merged_trimesh(cut_parts_nef); //cut_parts.front(); nef_to_single_trimesh(cut_parts_nef); //cut_parts.front();

    // cout << "Result of clipping" << endl;
    // vector<triangular_mesh> subs{m};
    // auto cut_parts = boolean_difference(part, subs);
    // vtk_debug_mesh(cut_part);

    vector<surface> cregions = outer_surfaces(cut_part);
    sort(begin(cregions), end(cregions),
	 [](const surface& l, const surface& r)
	 { return l.surface_area() < r.surface_area(); });
    reverse(begin(cregions), end(cregions));

    cout << "# of const orientation regions = " << cregions.size() << endl;
    // for (auto cregion : cregions) {
    //   vtk_debug_highlight_inds(cregion);
    // }

    std::vector<clamp_orientation> orients =
      all_stable_orientations_with_top_normal(cregions, v, n);

    vector<pair<clamp_orientation, homogeneous_transform>> orients_with_transforms;

    for (auto orient : orients) {
      homogeneous_transform t = mating_transform(cut_part, orient, v);
      orients_with_transforms.push_back(std::make_pair(orient, t));
    }

    cout << "# of orients for cregions = " << orients.size() << endl;

    return orients_with_transforms;
    
  }

  std::vector<clamp_orientation>
  all_viable_clamp_orientations(const std::vector<const surface*>& surfaces,
				const vice& v) {
    vector<clamp_orientation> orients =
      all_clamp_orientations(surfaces);

    DBG_ASSERT(orients.size() > 0);

    filter_stub_orientations(orients, v);

    DBG_ASSERT(orients.size() > 0);
    
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
    		  > v.max_opening_capacity();
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
    DBG_ASSERT(surfaces_left.size() > 0);
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

  boost::optional<clamp_orientation>
  find_orientation_by_normal_optional(const std::vector<clamp_orientation>& orients,
				      const point n) {
    auto r = find_if(begin(orients), end(orients),
		     [n](const clamp_orientation& s)
		     { return within_eps(angle_between(s.top_normal(), n), 0, 1.0); });

    if (r != end(orients)) {
      return *r;
    } else {
      return boost::none;
    }
  }

  clamp_orientation
  find_orientation_by_normal(const std::vector<clamp_orientation>& orients,
			     const point n) {
    auto r = find_orientation_by_normal_optional(orients, n);

    DBG_ASSERT(r);

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

  ublas::matrix<double> plane_matrix(const clamp_orientation& clamp_orient) {
    point base_n = clamp_orient.bottom_normal();
    point left_n = clamp_orient.left_normal();
    point right_n = clamp_orient.right_normal();

    ublas::matrix<double> p_mat(3, 3);
    p_mat(0, 0) = base_n.x;
    p_mat(0, 1) = base_n.y;
    p_mat(0, 2) = base_n.z;

    p_mat(1, 0) = left_n.x;
    p_mat(1, 1) = left_n.y;
    p_mat(1, 2) = left_n.z;

    p_mat(2, 0) = right_n.x;
    p_mat(2, 1) = right_n.y;
    p_mat(2, 2) = right_n.z;

    return p_mat;
  }

  ublas::vector<double> plane_d_vector(const clamp_orientation& clamp_orient) {
    ublas::vector<double> d_vec(3);
    d_vec(0) = clamp_orient.base_plane().d();
    d_vec(1) = clamp_orient.left_plane().d();
    d_vec(2) = clamp_orient.right_plane().d();

    return d_vec;
  }

  point part_zero_position(const clamp_orientation& clamp_orient) {  
    const ublas::matrix<double> plane_mat = plane_matrix(clamp_orient);
    ublas::vector<double> plane_d_vec = -1*plane_d_vector(clamp_orient);

    const ublas::matrix<double> p_inv = inverse(plane_mat);
    return from_vector(prod(p_inv, plane_d_vec));
  }

}
