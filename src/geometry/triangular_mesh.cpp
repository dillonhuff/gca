#include "geometry/triangular_mesh.h"

namespace gca {

  index_t find_index(point p, std::vector<point>& vertices, double tolerance) {
    for (unsigned i = 0; i < vertices.size(); i++) {
      if (within_eps(p, vertices[i])) { return i; }
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
    auto num_vertices = vertices.size();
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
	// TODO: Add proper triangle height computation
	return maybe<double>(vertices[t.v[0]].z);
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

}
