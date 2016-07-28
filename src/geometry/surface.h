#ifndef GCA_SURFACE_H
#define GCA_SURFACE_H

#include <boost/optional.hpp>

#include "geometry/triangular_mesh.h"

namespace gca {

  class surface {
  protected:
    const triangular_mesh* parent_mesh;
    vector<index_t> tri_indexes;

  public:
    inline bool contains(index_t ind) const {
      return std::binary_search(begin(tri_indexes), end(tri_indexes), ind);
    }

    std::vector<index_t> index_list() const {
      return tri_indexes;
    }

    bool contained_by_sorted(const std::vector<index_t>& inds) const {
      for (auto i : tri_indexes) {
	if (!binary_search(begin(inds), end(inds), i)) {
	  return false;
	}
      }
      return true;
    }

    bool contained_by(const surface& other) const {
      return contained_by_sorted(other.tri_indexes);
    }
    
    inline index_t front() const {
      return tri_indexes.front();
    }

    double surface_area() const {
      double total = 0.0;
      for (auto i : tri_indexes) {
	total += (parent_mesh->face_triangle(i)).area();
      }
      return total;
    }

    inline point face_orientation(index_t ind) const
    { return parent_mesh->face_orientation(ind); }

    inline triangle face_triangle(index_t ind) const
    { return get_parent_mesh().face_triangle(ind); }
    
    surface(const triangular_mesh* p_parent_mesh,
	    const vector<index_t>& p_tri_indexes) :
      parent_mesh(p_parent_mesh), tri_indexes(p_tri_indexes) {
      assert(tri_indexes.size() > 0);
      std::sort(begin(tri_indexes), end(tri_indexes));
    }

    inline const triangular_mesh& get_parent_mesh() const
    { return *parent_mesh; }

    bool orthogonal_to(const point n, double tol) const {
      return all_orthogonal_to(index_list(), get_parent_mesh(), n, tol);
    }

    bool parallel_to(const point n, double tol) const {
      return all_parallel_to(index_list(), get_parent_mesh(), n, tol);
    }

  };

  bool surfaces_share_edge(const unsigned i,
			   const unsigned j,
			   const std::vector<surface>& surfaces);

  bool surfaces_share_edge(const unsigned i,
			   const unsigned j,
			   const std::vector<surface*>& surfaces);
  
  bool surfaces_share_edge(const surface& l,
			   const surface& r);

  
  std::vector<index_t> surface_vertexes(const surface& s);

  bool orthogonal_flat_surfaces(const surface* l, const surface* r);
  bool parallel_flat_surfaces(const surface* l, const surface* r);
  std::vector<surface> outer_surfaces(const triangular_mesh& part);

  typedef std::vector<std::vector<index_t>> surface_list;

  surface merge_surfaces(const std::vector<surface>& surfaces);

  void remove_contained_surfaces(const std::vector<surface>& stable_surfaces,
				 std::vector<surface>& surfaces_to_cut);

  boost::optional<oriented_polygon>
  part_outline(std::vector<surface>* surfaces_to_cut);

  boost::optional<surface>
  part_outline_surface(std::vector<surface>* surfaces_to_cut,
		       const point n);

  typedef std::vector<unsigned> surface_group;

  boost::optional<surface>
  part_outline_surface(const triangular_mesh& m,
		       const point n);

  std::vector<surface> surfaces_to_cut(const triangular_mesh& part);

  bool
  share_edge(const std::vector<gca::edge>& edges,
	     const surface& l,
	     const surface& r);

  std::vector<surface_group>
  convex_surface_groups(const std::vector<surface*>& surfaces);

  std::vector<surface>
  constant_orientation_subsurfaces(const surface& surface);

  std::vector<surface>
  connected_vertical_surfaces(const triangular_mesh& m, const point n);
  
}

#endif
