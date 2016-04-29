#ifndef GCA_TRIANGULAR_MESH_H
#define GCA_TRIANGULAR_MESH_H

#include <numeric>

#include "geometry/box.h"
#include "geometry/triangle.h"
#include "geometry/trimesh.h"

namespace gca {

  class triangular_mesh {
  private:
    std::vector<point> vertices;
    std::vector<triangle_t> tri_vertices;
    std::vector<point> face_orientations;
    trimesh_t mesh;

  public:
    triangular_mesh(const std::vector<point>& vertices_p,
		    const std::vector<triangle_t>& triangles_p,
		    const std::vector<point>& face_orientations_p,
		    trimesh_t mesh_p) :
      vertices(vertices_p),
      tri_vertices(triangles_p),
      face_orientations(face_orientations_p),
      mesh(mesh_p) {}

    inline std::vector<index_t> face_indexes() const {
      std::vector<index_t> indices(tri_vertices.size());
      std::iota(begin(indices), end(indices), 0);
      return indices;
    }

    inline bool is_connected() const {
      return mesh.boundary_vertices().size() == 0;
    }

    inline point face_orientation(index_t i) const
    { return face_orientations[i]; }

    bool is_constant_orientation_vertex(const point p,
					double tolerance) const;

    inline box bounding_box() const { return bound_positions(vertices); }

    maybe<double> z_at(double x, double y) const;
    double z_at_unsafe(double x, double y) const;
    
    std::vector<triangle> triangle_list() {
      std::vector<triangle> ts;
      // TODO: Add real triangle -> normal map or compute normals
      point dummy_normal(1, 0, 0);
      for (unsigned i = 0; i < tri_vertices.size(); i++) {
	auto t = tri_vertices[i];
	ts.push_back(triangle(face_orientations[i],
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
