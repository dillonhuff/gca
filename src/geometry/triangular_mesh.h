#ifndef GCA_TRIANGULAR_MESH_H
#define GCA_TRIANGULAR_MESH_H

#include "geometry/triangle.h"
#include "geometry/trimesh.h"

namespace gca {

  class triangular_mesh {
  private:
    vector<point> vertices;
    vector<triangle_t> tri_vertices;
    trimesh_t mesh;

  public:

    triangular_mesh(const std::vector<point>& vertices_p,
		    const std::vector<triangle_t>& triangles_p,
		    trimesh_t mesh_p) :
      vertices(vertices_p), tri_vertices(triangles_p), mesh(mesh_p) {}
    
    std::vector<triangle> triangle_list() {
      std::vector<triangle> ts;
      // TODO: Add real triangle -> normal map or compute normals
      point dummy_normal(1, 0, 0);
      for (auto t : tri_vertices) {
	ts.push_back(triangle(dummy_normal,
			      vertices[t.i()],
			      vertices[t.j()],
			      vertices[t.k()]));
      }
      return ts;
    }
  };

  triangular_mesh make_mesh(const std::vector<triangle>& triangles,
			    double tolerance);
}
#endif
