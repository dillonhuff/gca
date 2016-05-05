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

    inline triangle_t triangle_vertices(const index_t vi) const {
      if (!(vi < tri_vertices.size())) {
	cout << "Error in triangular_mesh::triangle_vertices, ";
	cout << "!(" << vi << " < " << tri_vertices.size() << ")" << endl;
	assert(false);
      }
      return tri_vertices[vi];
    }

    inline std::vector<index_t> vertex_face_neighbors(const index_t vi) const {
      return mesh.vertex_face_neighbors(vi);
    }

    double surface_area() const {
      double total = 0.0;
      for (unsigned i = 0; i < tri_vertices.size(); i++) {
	total += face_triangle(i).area();
      }
      return total;
    }


    inline bool is_connected() const {
      return mesh.boundary_vertices().size() == 0;
    }

    inline point face_orientation(index_t i) const
    { return face_orientations[i]; }

    inline triangle face_triangle(index_t i) const {
      auto t = tri_vertices[i];
      return triangle(face_orientations[i],
		      vertices[t.i()],
		      vertices[t.j()],
		      vertices[t.k()]);
    }

    bool is_constant_orientation_vertex(const point p,
					double tolerance) const;

    inline box bounding_box() const { return bound_positions(vertices); }

    maybe<double> z_at(double x, double y) const;
    double z_at_unsafe(double x, double y) const;

    inline const vector<point>& vertex_list() const {
      return vertices;
    }

    std::vector<triangle_t> triangle_verts() const
    { return tri_vertices; }
    
    std::vector<triangle> triangle_list() const {
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

    template<typename F>
    triangular_mesh apply(F f) const {
      vector<point> tverts(vertices.size());
      transform(begin(vertices), end(vertices), begin(tverts), f);
      vector<point> torients(face_orientations.size());
      transform(begin(face_orientations), end(face_orientations),
		begin(torients),
		f);
      return triangular_mesh(tverts, tri_vertices, torients, mesh);
    }
  };

  triangular_mesh make_mesh(const std::vector<triangle>& triangles,
			    double tolerance);

    // NOTE: Assumes all triangles of s are coplanar, e.g. same normal
  bool is_outer_surface(const std::vector<index_t>& s, const triangular_mesh& part);

  std::vector<std::vector<index_t>>
  const_orientation_regions(const triangular_mesh& part);

  void
  transfer_face(index_t face_ind,
		std::vector<index_t>& old_face_inds,
		std::vector<index_t>& face_inds,
		std::vector<index_t>& remaining_vertex_inds,
		const triangular_mesh& part);

  template<typename S, typename N>
  std::vector<index_t> connected_region(vector<index_t>& face_indices,
					const triangular_mesh& part,
					S select_initial_face,
					N neighboring_faces) {
    assert(face_indices.size() > 0);
    sort(begin(face_indices), end(face_indices));
    vector<index_t> surface_face_inds;
    vector<index_t> remaining_vertex_inds;
    transfer_face(select_initial_face(part, face_indices),
		  face_indices,
		  surface_face_inds,
		  remaining_vertex_inds,
		  part);
    while (remaining_vertex_inds.size() > 0) {
      auto next_v = remaining_vertex_inds.back();
      remaining_vertex_inds.pop_back();
      for (auto f : neighboring_faces(part, next_v)) {
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

  triangular_mesh operator*(const matrix<3, 3>& m, const triangular_mesh& mesh);
  
}
#endif
