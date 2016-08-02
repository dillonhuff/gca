#include "geometry/triangular_mesh.h"
#include "utils/algorithm.h"

namespace gca {

  index_t
  vertex_index_of(const index_t ind, const triangle_t t1) {
    for (index_t i = 0; i < 3; i++) {
      if (t1.v[i] == ind) { return i; }
    }
    assert(false);
  }

  std::vector<index_t>
  edge_triangle_inds(const gca::edge e,
		     const triangular_mesh& m) {
    auto tl = m.vertex_face_neighbors(e.l);
    auto tr = m.vertex_face_neighbors(e.r);
    return intersection(tl, tr);
  }

  std::vector<gca::edge> non_manifold_edges(const triangular_mesh& m) {
    vector<gca::edge> nm_edges;
    for (auto e : m.edges()) {
      if (edge_triangle_inds(e, m).size() != 2) {
	nm_edges.push_back(e);
      }
    }
    return nm_edges;
  }
  
  bool triangular_mesh::winding_order_is_consistent() const {
    for (auto e : edges()) {
      auto tl = vertex_face_neighbors(e.l);
      auto tr = vertex_face_neighbors(e.r);
      auto tris = intersection(tl, tr);
      assert(tris.size() == 2);
      triangle_t t1 = triangle_vertices(tris[0]);
      triangle_t t2 = triangle_vertices(tris[1]);
      index_t t1l = vertex_index_of(e.l, t1);
      index_t t1r = vertex_index_of(e.r, t1);

      index_t t2l = vertex_index_of(e.l, t2);
      index_t t2r = vertex_index_of(e.r, t2);

      if (t1l == t2l && t1r == t2r) {
	return false;
      }
    }
    return true;
  }

  index_t find_index(point p, std::vector<point>& vertices, double tolerance) {
    for (unsigned i = 0; i < vertices.size(); i++) {
      if (within_eps(p, vertices[i], tolerance)) { return i; }
    }
    vertices.push_back(p);
    return vertices.size() - 1;
  }

  index_t find_index_with_fail(point p,
			       const std::vector<point>& vertices,
			       double tolerance) {
    for (unsigned i = 0; i < vertices.size(); i++) {
      if (within_eps(p, vertices[i])) { return i; }
    }
    assert(false);
  }
  
  void
  fill_vertex_triangles(const std::vector<triangle>& triangles,
			std::vector<triangle_t>& vertex_triangles,
			std::vector<point>& vertices,
			double tolerance) {
    for (auto t : triangles) {
      auto v1i = find_index(t.v1, vertices, tolerance);
      auto v2i = find_index(t.v2, vertices, tolerance);
      auto v3i = find_index(t.v3, vertices, tolerance);
      triangle_t tr;
      tr.v[0] = v1i;
      tr.v[1] = v2i;
      tr.v[2] = v3i;
      vertex_triangles.push_back(tr);
    }
  }

  bool triangular_mesh::is_constant_orientation_vertex(const point p,
						       double tolerance) const {
    index_t i = find_index_with_fail(p, vertices, 0.001);
    auto face_neighbors = mesh.vertex_face_neighbors(i);
    vector<point> face_orientations;
    for (auto fi : face_neighbors) {
      face_orientations.push_back(face_orientation(fi));
    }
    for (auto lo : face_orientations) {
      for (auto ro : face_orientations) {
	if (!within_eps(angle_between(lo, ro), 0.0, tolerance)) {
	  return false;
	}
      }
    }
    return true;
  }

  triangular_mesh make_mesh(const std::vector<triangle>& triangles,
			    double tolerance) {
    std::vector<point> vertices;
    std::vector<triangle_t> vertex_triangles;
    fill_vertex_triangles(triangles, vertex_triangles, vertices, tolerance);
    std::vector<point> face_orientations(triangles.size());
    transform(begin(triangles), end(triangles), begin(face_orientations),
	      [](const triangle t) { return t.normal; });
    std::vector<edge_t> edges;
    unordered_edges_from_triangles(vertex_triangles.size(),
				   &vertex_triangles[0],
				   edges);
    trimesh_t mesh;
    mesh.build(vertices.size(),
	       triangles.size(),
	       &vertex_triangles[0],
	       edges.size(),
	       &edges[0]);
    return triangular_mesh(vertices, vertex_triangles, face_orientations, mesh);
  }

  double sign(point p1, point p2, point p3) {
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
  }

  bool point_in_triangle_2d(point pt, point v1, point v2, point v3) {
    bool b1, b2, b3;

    b1 = sign(pt, v1, v2) < 0.0f;
    b2 = sign(pt, v2, v3) < 0.0f;
    b3 = sign(pt, v3, v1) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
  }

  maybe<double> triangular_mesh::z_at(double x, double y) const {
    for (unsigned i = 0; i < tri_vertices.size(); i++) {
      auto t = tri_vertices[i];
      auto orient = face_orientations[i];
      if (point_in_triangle_2d(point(x, y, 0),
			       vertices[t.v[0]],
			       vertices[t.v[1]],
			       vertices[t.v[2]]) &&
	  orient.z > 0.01) {
	triangle tr = face_triangle(i);
	return maybe<double>(gca::z_at(tr, x, y));
      }
    }
    return maybe<double>();
  }

  double triangular_mesh::z_at_unsafe(double x, double y) const {
    maybe<double> z = z_at(x, y);
    if (!(z.just)) {
      cout << "ERROR: No z value for point: " << point(x, y, 0) << endl;
      assert(false);
    }
    return z.t;
  }

  // NOTE: Assumes all triangles of s are coplanar, e.g. same normal
  bool is_outer_surface(const vector<index_t>& s, const triangular_mesh& part) {
    assert(s.size() > 0);
    triangle plane_rep = part.face_triangle(s.front());
    point v1 = plane_rep.v1;
    point n = plane_rep.normal.normalize();
    bool all_neg = true;
    bool all_pos = true;
    double d = -(v1.dot(n));

    vector<index_t> s_inds = s;
    sort(begin(s_inds), end(s_inds));

    //    for (auto p : part.vertex_list()) {
    for (auto i : inds(part.vertex_list())) {
      if (!binary_search(begin(s_inds), end(s_inds), i)) {
	point p = part.vertex(i);
	double sgn = n.dot(p) + d; // - v1);
	// TODO: Eliminate ad hoc tolerance
	if (sgn > 0.01) {
	  all_neg = false;
	}
	// TODO: Eliminate ad hoc tolerance
	if (sgn < 0.01) {
	  all_pos = false;
	}
	if (!all_neg && !all_pos) { return false; }
      }
    }
    return true;
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

  index_t back_face(const triangular_mesh&, std::vector<index_t>& face_indices)
  {  return face_indices.back(); }

  std::vector<index_t> all_neighbors(const triangular_mesh& part,
				     const index_t next_vertex) {
    return part.vertex_face_neighbors(next_vertex);
  }

  std::vector<vector<index_t>>
  connect_regions(std::vector<index_t>& indices,
		  const triangular_mesh& part) {
    assert(indices.size() > 0);
    vector<vector<index_t>> connected_regions;
    while (indices.size() > 0) {
      connected_regions.push_back(connected_region(indices,
						   part,
						   back_face,
						   all_neighbors));
    }
    return connected_regions;
  }

  std::vector<std::vector<index_t>>
  const_orientation_regions(const triangular_mesh& part) {
    vector<index_t> inds = part.face_indexes();
    return normal_delta_regions(inds, part, 0.0001);
  }

  double diameter(const point normal, const triangular_mesh& m) {
    return diameter(normal, m.vertex_list());
  }

  double max_in_dir(const triangular_mesh& mesh,
		    const point dir) {
    return max_distance_along(mesh.vertex_list(), dir);
  }

  double min_in_dir(const triangular_mesh& mesh,
		    const point dir) {
    return min_distance_along(mesh.vertex_list(), dir);
  }

  point min_point_in_dir(const triangular_mesh& mesh, const point dir) {
    return min_along(mesh.vertex_list(), dir);
  }
  
  point max_point_in_dir(const triangular_mesh& mesh, const point dir) {
    return max_along(mesh.vertex_list(), dir);
  }


  std::vector<index_t> select_visible_triangles(const triangular_mesh& mesh) {
    std::vector<index_t> tris;
    for (auto i : mesh.face_indexes()) {
      triangle t = mesh.face_triangle(i);
      if (is_upward_facing(t, 0.01)) {
	tris.push_back(i);
      }
    }
    return tris;
  }

  maybe<double> z_at(const double x,
		     const double y,
		     const std::vector<index_t>& faces,
		     const triangular_mesh& mesh) {
    for (auto i : faces) {
      auto t = mesh.face_triangle(i);
      auto orient = t.normal;
      if (point_in_triangle_2d(point(x, y, 0),
			       t.v1,
			       t.v2,
			       t.v3) &&
	  orient.z > 0.01) {
	return maybe<double>(z_at(t, x, y));
      }
    }
    return maybe<double>();
  }

  double z_at_unsafe(const double x,
		     const double y,
		     const std::vector<index_t>& faces,
		     const triangular_mesh& mesh) {
    maybe<double> z = z_at(x, y, faces, mesh);
    if (!(z.just)) {
      cout << "ERROR: No z value for point: " << point(x, y, 0) << endl;
      assert(false);
    }
    return z.t;
  }

  std::vector<std::vector<index_t>>
  normal_delta_regions(vector<index_t>& indices,
		       const triangular_mesh& mesh,
		       double delta_degrees) {
    vector<vector<index_t>> connected_regions;
    auto within_delta = [delta_degrees](const index_t f,
					const index_t i,
					const triangular_mesh& m) {
      return within_eps(angle_between(m.face_triangle(f).normal,
				      m.face_triangle(i).normal),
			0,
			delta_degrees);
    };
    auto back_face_vec = [](const vector<index_t>& faces,
			    const triangular_mesh& mesh) {
      return vector<index_t>{faces.back()};
    };
    while (indices.size() > 0) {
      connected_regions.push_back(region(indices,
					 mesh,
					 back_face_vec,
					 within_delta));
    }
    return connected_regions;
  }

  bool share_edge(const index_t l,
		  const index_t r,
		  const triangular_mesh& part) {
    auto tl = part.triangle_vertices(l);
    auto tr = part.triangle_vertices(r);
    int num_eq = 0;
    for (unsigned i = 0; i < 3; i++) {
      for (unsigned j = 0; j < 3; j++) {
	num_eq += (tl.v[i] == tr.v[j]) ? 1 : 0;
      }
    }
    return num_eq > 1;
  }
  
  bool share_edge(const std::vector<index_t>& l_faces,
		  const std::vector<index_t>& r_faces,
		  const triangular_mesh& part) {
    for (auto l : l_faces) {
      for (auto r : r_faces) {
	if (share_edge(l, r, part)) {
	  return true;
	}
      }
    }
    return false;
  }

  std::vector<std::vector<index_t>>
  merge_connected_surfaces(const std::vector<std::vector<index_t>>& surfaces,
			   const triangular_mesh& part) {
    assert(surfaces.size() > 0);
    vector<vector<unsigned>> components =
      connected_components_by(surfaces,
			      [part](const vector<index_t>& l,
				     const vector<index_t>& r) {
				return share_edge(l, r, part);
			      });
    vector<vector<index_t>> merged;
    for (auto component : components) {
      vector<index_t> s;
      for (auto i : component) {
	concat(s, surfaces[i]);
      }
      merged.push_back(s);
    }
    return merged;
  }

  triangular_mesh triangulate(const oriented_polygon& p) {
    cout << "Making triangular mesh" << endl;
    auto tris = triangulate_polygon(p);
    return make_mesh(tris, 0.001);
  }

  bool any_vertex_in(const triangle_t tri,
		     const std::vector<index_t>& inds) {
    if (binary_search(begin(inds), end(inds), tri.v[0])) {
      return true;
    }
    if (binary_search(begin(inds), end(inds), tri.v[1])) {
      return true;
    }
    if (binary_search(begin(inds), end(inds), tri.v[2])) {
      return true;
    }
    return false;
  }

  // TODO: Really should not be oriented polygons
  vector<oriented_polygon> mesh_bounds(const vector<index_t>& faces,
				       const triangular_mesh& mesh) {
    vector<oriented_polygon> ps;
    if (faces.size() == 0) {
      return ps;
    }
    point normal = mesh.face_orientation(faces.front());
    typedef pair<index_t, index_t> iline;
    vector<iline> tri_lines;
    for (auto i : faces) {
      auto t = mesh.triangle_vertices(i);
      tri_lines.push_back(iline(t.v[0], t.v[1]));
      tri_lines.push_back(iline(t.v[1], t.v[2]));
      tri_lines.push_back(iline(t.v[2], t.v[0]));
    }

    // TODO: Change to sort and count, maybe add to system/algorithm?
    vector<iline> no_ds;
    for (auto l : tri_lines) {
      int count = 0;
      for (auto r : tri_lines) {
	if ((l.first == r.first && l.second == r.second) ||
	    (l.first == r.second && l.second == r.first)) {
	  count++;
	}
      }
      assert(count > 0);
      if (count == 1) {
	no_ds.push_back(l);
      }
    }

    vector<line> no_dups;
    for (auto l : no_ds) {
      no_dups.push_back(line(mesh.vertex(l.first), mesh.vertex(l.second)));
    }

    return unordered_segments_to_polygons(normal, no_dups);
  }

  bool all_orthogonal_to(const vector<index_t>& triangles,
			 const triangular_mesh& mesh,
			 const point n,
			 const double tolerance) {
    for (auto i : triangles) {
      auto t = mesh.face_triangle(i);
      if (!within_eps(angle_between(t.normal, n), 90, tolerance)) {
	return false;
      }
    }
    return true;
  }

  bool all_parallel_to(const vector<index_t>& triangles,
			 const triangular_mesh& mesh,
			 const point n,
			 const double tolerance) {
    for (auto i : triangles) {
      auto t = mesh.face_triangle(i);
      if (!within_eps(angle_between(t.normal, n), 0.0, tolerance)) {
	return false;
      }
    }
    return true;
  }
  
  bool all_normals_below(const vector<index_t>& triangles,
			 const triangular_mesh& mesh,
			 const double v) {
    for (auto i : triangles) {
      auto t = mesh.face_triangle(i);
      if (t.normal.normalize().z > v) {
	return false;
      }
    }
    return true;
  }

  void filter_vertical_surfaces(std::vector<std::vector<index_t>>& delta_regions,
				const triangular_mesh& mesh) {
    delete_if(delta_regions,
    	      [&mesh](const vector<index_t>& surface)
    	      { return all_orthogonal_to(surface, mesh, point(0, 0, 1), 5.0); });
    delete_if(delta_regions,
    	      [&mesh](const vector<index_t>& surface)
    	      { return all_normals_below(surface, mesh, -0.1); });
  }

  bool operator==(const edge x, const edge y) {
    return (x.l == y.l && x.r == y.r) || (x.r == y.l && x.l == y.r);
  }

  bool share_endpoint(const edge x, const edge y) {
    return x.l == y.l || x.l == y.r || x.r == y.r || x.r == y.l;
  }

  double dihedral_angle(const gca::edge e, const triangular_mesh& m) {
    auto tl = m.vertex_face_neighbors(e.l);
    auto tr = m.vertex_face_neighbors(e.r);
    auto tris = intersection(tl, tr);
    assert(tris.size() == 2);
    point n1 = m.face_orientation(tris[0]);
    point n2 = m.face_orientation(tris[1]);
    return angle_between(n1, n2);
  }
  
  std::vector<gca::edge>
  convex_edges(const triangular_mesh& m) {
    vector<gca::edge> c_edges;
    for (auto e : m.edges()) {
      if (dihedral_angle(e, m) < 180) {
	c_edges.push_back(e);
      }
    }
    return c_edges;
  }

  oriented_polygon max_area_outline(const std::vector<index_t>& inds,
				    const triangular_mesh& m) {
    auto part_outlines = mesh_bounds(inds, m);

    assert(part_outlines.size() > 0);

    oriented_polygon part_outline =
      *(max_element(begin(part_outlines), end(part_outlines),
		    [](const oriented_polygon& l,
		       const oriented_polygon& r)
      { return area(l) < area(r); }));

    return part_outline;
  }

}
