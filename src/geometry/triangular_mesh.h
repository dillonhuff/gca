#ifndef GCA_TRIANGULAR_MESH_H
#define GCA_TRIANGULAR_MESH_H

#include <unordered_set>
#include <numeric>

#include "geometry/box.h"
#include "geometry/triangle.h"
#include "geometry/trimesh.h"
#include "utils/algorithm.h"

namespace gca {

  struct edge {
    index_t l;
    index_t r;

    edge(const index_t p_l, const index_t p_r) : l(p_l), r(p_r) {}
  };

  bool operator==(const edge x, const edge y);
  bool share_endpoint(const edge x, const edge y);
}

namespace std {
  template <>
  struct hash<gca::edge> {
    size_t operator()(gca::edge const& x) const noexcept {
      return 13 + std::hash<gca::index_t>()(x.l) + 13*std::hash<gca::index_t>()(x.r);
    }
  };  
}

namespace gca {

  class triangular_mesh {
  private:
    std::vector<point> vertices;
    std::vector<triangle_t> tri_vertices;
    std::vector<point> face_orientations;
    trimesh_t mesh;

  public:
    triangular_mesh() {}
    
    triangular_mesh(const std::vector<point>& vertices_p,
		    const std::vector<triangle_t>& triangles_p,
		    const std::vector<point>& face_orientations_p,
		    trimesh_t mesh_p) :
      vertices(vertices_p),
      tri_vertices(triangles_p),
      face_orientations(face_orientations_p),
      mesh(mesh_p) {}

    std::vector<edge> edges() const {
      std::vector<edge> edges;
      unordered_set<edge> edge_set;
      for (index_t i = 0; i < mesh.num_halfedges(); i++) {
	auto p = mesh.he_index2directed_edge(i);
	if (!elem(edge(p.first, p.second), edge_set) &&
	    !elem(edge(p.second, p.first), edge_set)) {
	  edges.push_back(edge(p.first, p.second));
	  edge_set.insert(edge(p.first, p.second));
	}
      }
      return edges;
    }

    bool winding_order_is_consistent() const;

    inline std::vector<index_t> face_indexes() const {
      std::vector<index_t> indices(tri_vertices.size());
      std::iota(begin(indices), end(indices), 0);
      return indices;
    }

    inline std::vector<index_t> face_face_neighbors(const index_t i) const {
      triangle_t t = triangle_vertices(i);
      std::vector<index_t> face_neighbors;
      concat(face_neighbors, mesh.vertex_face_neighbors(t.v[0]));
      concat(face_neighbors, mesh.vertex_face_neighbors(t.v[1]));
      concat(face_neighbors, mesh.vertex_face_neighbors(t.v[2]));
      sort(begin(face_neighbors), end(face_neighbors));
      unique(begin(face_neighbors), end(face_neighbors));
      return face_neighbors;
    }
    
    inline point vertex(const index_t i) const {
      return vertices[i];
    }
    
    inline std::vector<index_t> vertex_indexes() const {
      std::vector<index_t> indices(vertices.size());
      std::iota(begin(indices), end(indices), 0);
      return indices;
    }

    std::vector<index_t>
    edge_triangle_inds(const gca::edge e,
		       const triangular_mesh& m);
    
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

    std::vector<index_t>
    edge_face_neighbors(const gca::edge e) const;
  
    
    inline bool is_connected() const {
      return mesh.boundary_vertices().size() == 0;
    }

    inline point face_orientation(index_t i) const {
      //      return face_orientations[i];
      auto t = tri_vertices[i];
      return cross(vertex(t.v[2]) - vertex(t.v[0]),
		   vertex(t.v[1]) - vertex(t.v[0])).normalize();
    }

    inline triangle face_triangle(index_t i) const {
      auto t = tri_vertices[i];
      return triangle(face_orientation(i),
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
	ts.push_back(triangle(face_orientation(i),
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

    template<typename F>
    triangular_mesh apply_to_vertices(F f) const {
      vector<point> tverts(vertices.size());
      transform(begin(vertices), end(vertices), begin(tverts), f);
      return triangular_mesh(tverts, tri_vertices, face_orientations, mesh);
    }

  };

  void make_mesh(const std::vector<triangle>& triangles,
		 triangular_mesh* dest,
		 double tolerance);

  triangular_mesh make_mesh_no_winding_check(const std::vector<triangle>& triangles,
					     double tolerance);
  
  
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

  template<typename S, typename B>
  std::vector<index_t> region(vector<index_t>& face_indices,
			      const triangular_mesh& part,
			      S select_initial_faces,
			      B is_neighbor) {
    assert(face_indices.size() > 0);
    sort(begin(face_indices), end(face_indices));
    vector<index_t> surface_face_inds = select_initial_faces(face_indices, part);
    vector<index_t> unchecked_face_inds = surface_face_inds;
    subtract(face_indices, surface_face_inds);
    while (unchecked_face_inds.size() > 0) {
      auto next_face = unchecked_face_inds.back();
      unchecked_face_inds.pop_back();
      for (auto f : part.face_face_neighbors(next_face)) {
	if (binary_search(begin(face_indices), end(face_indices), f) &&
	    is_neighbor(next_face, f, part)) {
	  remove(f, face_indices);
	  surface_face_inds.push_back(f);
	  unchecked_face_inds.push_back(f);
	}
      }
    }
    return surface_face_inds;
  }

  std::ostream& operator<<(std::ostream& out, const edge e);

  std::vector<gca::edge>
  non_manifold_edges(const triangular_mesh& m);

  
  double diameter(const point normal, const triangular_mesh& m);

  double min_in_dir(const triangular_mesh& mesh, const point dir);
  double max_in_dir(const triangular_mesh& mesh, const point dir);

  point min_point_in_dir(const triangular_mesh& mesh, const point dir);
  point max_point_in_dir(const triangular_mesh& mesh, const point dir);
  
  std::vector<index_t> select_visible_triangles(const triangular_mesh& mesh);

  std::vector<vector<index_t>>
  connect_regions(std::vector<index_t>& indices,
		  const triangular_mesh& part);

  maybe<double> z_at(const double x,
		     const double y,
		     const std::vector<index_t>& faces,
		     const triangular_mesh& mesh);

  double z_at_unsafe(const double x,
		     const double y,
		     const std::vector<index_t>& faces,
		     const triangular_mesh& mesh);

  std::vector<std::vector<index_t>>
  normal_delta_regions(vector<index_t>& indices,
		       const triangular_mesh& mesh,
		       double delta_degrees);

  bool share_edge(const index_t l,
		  const index_t r,
		  const triangular_mesh& part);

  bool share_edge(const std::vector<index_t>& l_faces,
		  const std::vector<index_t>& r_faces,
		  const triangular_mesh& part);

  std::vector<std::vector<index_t>>
  merge_connected_surfaces(const std::vector<std::vector<index_t>>& surfaces,
			   const triangular_mesh& part);
  
  triangular_mesh triangulate(const oriented_polygon& p);

  bool any_vertex_in(const triangle_t tri,
		     const std::vector<index_t>& inds);

  vector<oriented_polygon> mesh_bounds(const vector<index_t>& faces,
				       const triangular_mesh& mesh);

  void filter_vertical_surfaces(std::vector<std::vector<index_t>>& delta_regions,
				const triangular_mesh& mesh);

  bool all_normals_below(const vector<index_t>& triangles,
			 const triangular_mesh& mesh,
			 const double v);

  bool all_orthogonal_to(const vector<index_t>& triangles,
			 const triangular_mesh& mesh,
			 const point n,
			 const double tolerance);

  bool all_parallel_to(const vector<index_t>& triangles,
		       const triangular_mesh& mesh,
		       const point n,
		       const double tolerance);
  
  std::vector<gca::edge>
  convex_edges(const triangular_mesh& m);

  double dihedral_angle(const gca::edge e, const triangular_mesh& m);

  oriented_polygon max_area_outline(const std::vector<index_t>& inds,
				    const triangular_mesh& m);

  std::ostream& operator<<(std::ostream& out, const triangle_t t);

}

#endif
