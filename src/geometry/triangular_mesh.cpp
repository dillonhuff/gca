#include "geometry/triangular_mesh.h"

namespace gca {

  index_t find_index(point p, std::vector<point>& vertices, double tolerance) {
    for (unsigned i = 0; i < vertices.size(); i++) {
      if (within_eps(p, vertices[i])) { return i; }
    }
    vertices.push_back(p);
    return vertices.size() - 1;
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

  triangular_mesh make_mesh(const std::vector<triangle>& triangles,
			    double tolerance) {
    std::vector<point> vertices;
    std::vector<triangle_t> vertex_triangles;
    fill_vertex_triangles(triangles, vertex_triangles, vertices, tolerance);
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
    return triangular_mesh(vertices, vertex_triangles, mesh);
  }

}
