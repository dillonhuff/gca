#include <set>

#include "synthesis/mesh_to_gcode.h"
#include "system/algorithm.h"

namespace gca {

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const triangular_mesh& workpiece_mesh) {
    vector<point> normals;
    normals.push_back(point(1, 0, 0));
    normals.push_back(point(-1, 0, 0));
    normals.push_back(point(0, 1, 0));
    normals.push_back(point(0, -1, 0));
    normals.push_back(point(0, 0, 1));
    normals.push_back(point(0, 0, -1));

    for (auto& sf : part_surfaces) {
      for (auto n : normals) {
	if (within_eps(angle_between(n, sf.face_orientation(sf.front())), 0, 0.001)) {
	  sf.set_SA();
	}
      }
    }
  }

  // TODO: Change to actually align instead of just making surfaces
  // orthogonal to axes
  triangular_mesh align_workpiece(const std::vector<surface>& part_surfaces,
				  const workpiece_dimensions w_dims) {
    point p0(0, 0, 0);
    point p1(w_dims.x, 0, 0);
    point p2(w_dims.x, w_dims.y, 0);
    point p3(0, w_dims.y, 0);
    point p4(0, 0, w_dims.z);
    point p5(w_dims.x, 0, w_dims.z);
    point p6(w_dims.x, w_dims.y, w_dims.z);
    point p7(0, w_dims.y, w_dims.z);

    point n0(1, 0, 0);
    point n1(-1, 0, 0);

    point n2(0, 1, 0);
    point n3(0, -1, 0);

    point n4(0, 0, 1);
    point n5(0, 0, -1);

    vector<triangle> ts;
    ts.push_back(triangle(n4, p4, p7, p6));
    ts.push_back(triangle(n4, p5, p4, p6));
    
    // ts.push_back(triangle(n0, p1, p5, p6));
    // ts.push_back(triangle(n0, p1, p2, p6));

    // ts.push_back(triangle(n1, p3, p0, p4));
    // ts.push_back(triangle(n1, p0, p3, p7));

    // ts.push_back(triangle(n2, p3, p2, p7));
    // ts.push_back(triangle(n2, p2, p3, p6));

    // ts.push_back(triangle(n3, p4, p5, p1));
    // ts.push_back(triangle(n3, p1, p0, p4));

    // ts.push_back(triangle(n5, p0, p1, p2));
    // ts.push_back(triangle(n5, p0, p3, p2));

    return make_mesh(ts, 0.001);
  }

  bool any_SA_surface_contains(index_t i,
			       const std::vector<surface>& surfaces) {
    for (auto surface : surfaces) {
      if (surface.is_SA() && surface.contains(i)) { return true; }
    }
    return false;
  }

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices) {
    delete_if(indices,
	      [&surfaces](index_t i)
	      { return any_SA_surface_contains(i, surfaces); });
  }

  void
  transfer_face(index_t face_ind,
		std::vector<index_t>& old_face_inds,
		std::vector<index_t>& face_inds,
		std::vector<index_t>& remaining_vertex_inds,
		const triangular_mesh& part) {
    remove(face_ind, old_face_inds);
    face_inds.push_back(face_ind);
    triangle_t t = part.triangle_vertices(face_ind);
    remaining_vertex_inds.push_back(t.v[0]);
    remaining_vertex_inds.push_back(t.v[1]);
    remaining_vertex_inds.push_back(t.v[2]);
  }

  std::vector<index_t> connected_region(vector<index_t>& face_indices,
					const triangular_mesh& part) {
    assert(face_indices.size() > 0);
    sort(begin(face_indices), end(face_indices));
    vector<index_t> surface_face_inds;
    vector<index_t> remaining_vertex_inds;
    transfer_face(face_indices.back(),
		  face_indices,
		  surface_face_inds,
		  remaining_vertex_inds,
		  part);
    while (remaining_vertex_inds.size() > 0) {
      auto next_v = remaining_vertex_inds.back();
      remaining_vertex_inds.pop_back();
      for (auto f : part.vertex_face_neighbors(next_v)) {
	if (binary_search(begin(face_indices), end(face_indices), f)) {
	  transfer_face(f,
			face_indices,
			surface_face_inds,
			remaining_vertex_inds, part);
	}
      }
    }
    return surface_face_inds;
  }

  std::vector<vector<index_t>>
  connect_regions(vector<index_t>& indices,
		  const triangular_mesh& part) {
    assert(indices.size() > 0);
    vector<vector<index_t>> connected_regions;
    while (indices.size() > 0) {
      connected_regions.push_back(connected_region(indices, part));
    }
    return connected_regions;
  }

  std::vector<std::vector<index_t>>
  const_orientation_regions(const triangular_mesh& part) {
    auto faces = part.face_indexes();
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).x < part.face_orientation(r).x; });
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).y < part.face_orientation(r).y; });
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).z < part.face_orientation(r).z; });
    vector<vector<index_t>> const_orient_face_indices;
    split_by(faces,
	     const_orient_face_indices,
	     [&part](index_t l, index_t r)
	     { return within_eps(part.face_orientation(l), part.face_orientation(r)); });
    vector<vector<index_t>> const_connected_regions;
    for (auto r : const_orient_face_indices) {
      vector<vector<index_t>> connected_regions = connect_regions(r, part);
      const_connected_regions.insert(end(const_connected_regions),
				     begin(connected_regions),
				     end(connected_regions));
    }
    return const_connected_regions;
  }

  point project_onto(point p, point proj_d) {
    point proj_dir = proj_d.normalize();
    return (p.dot(proj_dir))*proj_dir;
  }

  double distance_along(point normal, const triangle t) {
    point p = t.v1;
    point dir = normal.normalize();
    return ((p.dot(dir))*dir).len();
  }

  std::vector<index_t> outermost_by(point normal,
				    const std::vector<index_t> faces,
				    const triangular_mesh& part,
				    double tolerance) {
    assert(faces.size() > 0);
    auto f = faces;
    sort(begin(f), end(f),
	 [&part, normal](index_t l, index_t r)
	 { return distance_along(normal, part.face_triangle(l)) >
	   distance_along(normal, part.face_triangle(r)); });
    double outer_dist = distance_along(normal, part.face_triangle(f.front()));
    take_while(f,
	       [&part, normal, outer_dist, tolerance](index_t i)
	       { return within_eps(outer_dist,
				   distance_along(normal, part.face_triangle(i)),
				   tolerance); });
    assert(f.size() > 0);
    return f;
  }

  // NOTE: Assumes all triangles of s are coplanar, e.g. same normal
  bool is_outer_surface(const vector<index_t>& s, const triangular_mesh& part) {
    assert(s.size() > 0);
    triangle plane_rep = part.face_triangle(s.front());
    point v1 = plane_rep.v1;
    point n = plane_rep.normal.normalize();
    bool all_neg = true;
    bool all_pos = true;
    for (auto p : part.vertex_list()) {
      double sgn = n.dot(p - v1);
      if (sgn > 0) {
    	all_neg = false;
      }
      if (sgn < 0) {
	all_pos = false;
      }
      if (!all_neg && !all_pos) { return false; }
    }
    return true;
  }

  std::vector<surface> outer_surfaces(const triangular_mesh& part) {
    auto const_orient_face_indices = const_orientation_regions(part);
    cout << "# const orientation regions = " << const_orient_face_indices.size() << endl;
    vector<surface> surfaces;
    for (auto f : const_orient_face_indices) {
      assert(f.size() > 0);
      point face_normal = part.face_orientation(f.front());
      if (is_outer_surface(f, part)) {
	surfaces.push_back(surface(&part, f));
      }
    }
    return surfaces;
  }

  // TODO: Replace this dummy
  std::vector<gcode_program>
  workpiece_clipping_programs(const triangular_mesh& workpiece_mesh,
			      const triangular_mesh& part_mesh) {
    vector<gcode_program> clip_progs;
    for (int i = 0; i < 3; i++) {
      clip_progs.push_back(gcode_program());
      clip_progs.push_back(gcode_program());
    }
    return clip_progs;
  }

  bool face_is_millable_from(index_t i,
			     const stock_orientation& orient,
			     const triangular_mesh& part_mesh) {
    point top_normal = orient.top_normal();
    point i_normal = part_mesh.face_orientation(i);
    return within_eps(angle_between(top_normal, i_normal), 0, 90);
  }

  bool orthogonal_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 90, 0.1);
  }

  bool parallel_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 180, 0.1);
  }
  
  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces) {
    vector<stock_orientation> orients;
    for (unsigned j = 0; j < surfaces.size(); j++) {
      const surface* next_left = &(surfaces[j]);
      for (unsigned k = 0; k < surfaces.size(); k++) {
	const surface* next_right = &(surfaces[k]);
	if (parallel_flat_surfaces(next_right, next_left)) {
	  for (unsigned l = 0; l < surfaces.size(); l++) {
	    const surface* next_bottom = &(surfaces[l]);
	    if (orthogonal_flat_surfaces(next_bottom, next_left)) {
	      orients.push_back(stock_orientation(next_left,
						  next_right,
						  next_bottom));
	    }
	  }
	}
      }
    }
    assert(orients.size() > 0);
    return orients;
  }

  std::vector<stock_orientation>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& surfaces,
		      std::vector<index_t>& faces_to_cut) {
    vector<stock_orientation> all_orients = all_stable_orientations(surfaces);
    vector<stock_orientation> orients;
    while (faces_to_cut.size() > 0) {
      assert(all_orients.size() > 0);
      auto next_orient = all_orients.back();
      all_orients.pop_back();
      unsigned old_size = faces_to_cut.size();
      // for (auto f : faces_to_cut) {
      // 	cout << part_mesh.face_orientation(f) << endl;
      // }
      delete_if(faces_to_cut,
		[&part_mesh, &next_orient](index_t i)
		{ return face_is_millable_from(i, next_orient, part_mesh); });
      if (faces_to_cut.size() != old_size) {
	cout << "Faces left = " << faces_to_cut.size() << endl;
	orients.push_back(next_orient);
      }
    }
    return orients;
  }

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& part_mesh,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece_dimensions w_dims) {
    auto part_ss = outer_surfaces(part_mesh);
    auto workpiece_mesh = align_workpiece(part_ss, w_dims);
    classify_part_surfaces(part_ss, workpiece_mesh);
    //Hack to prevent any unusual surfaces from escaping
    delete_if(part_ss,
    	      [](const surface& s)
    	      { return !s.is_SA(); });
    vector<index_t> face_inds = part_mesh.face_indexes();
    cout << "# initial faces = " << face_inds.size() << endl;
    remove_SA_surfaces(part_ss, face_inds);
    vector<gcode_program> ps =
      workpiece_clipping_programs(workpiece_mesh, part_mesh);
    cout << "# faces left = " << face_inds.size() << endl;
    vector<stock_orientation> orients =
      orientations_to_cut(part_mesh, part_ss, face_inds);
    for (auto orient : orients) {
      cout << "top normal = " << orient.top_normal() << endl;
      ps.push_back(gcode_program());
    }
    return ps;
  }
}
