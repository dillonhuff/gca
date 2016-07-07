#ifndef GCA_SURFACE_H
#define GCA_SURFACE_H

#include "geometry/triangular_mesh.h"

namespace gca {

  class surface {
  protected:
    const triangular_mesh* parent_mesh;
    vector<index_t> tri_indexes;
    bool SA;

  public:
    inline bool contains(index_t ind) {
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
    
    inline bool is_SA() const { return SA; }
    inline void set_SA() { SA = true; }

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

    surface(const triangular_mesh* p_parent_mesh,
	    const vector<index_t>& p_tri_indexes) :
      parent_mesh(p_parent_mesh), tri_indexes(p_tri_indexes), SA(false) {
      assert(tri_indexes.size() > 0);
      std::sort(begin(tri_indexes), end(tri_indexes));
    }

    inline const triangular_mesh& get_parent_mesh() const
    { return *parent_mesh; }

    bool orthogonal_to(const point n, double tol) const {
      return all_orthogonal_to(index_list(), get_parent_mesh(), n, tol);
    }
  };

  bool surfaces_share_edge(const unsigned i,
			   const unsigned j,
			   const std::vector<surface>& surfaces);

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
  			  std::vector<index_t>& indices);

  std::vector<index_t> surface_vertexes(const surface& s);

  bool orthogonal_flat_surfaces(const surface* l, const surface* r);
  bool parallel_flat_surfaces(const surface* l, const surface* r);
  std::vector<surface> outer_surfaces(const triangular_mesh& part);

  typedef std::vector<std::vector<index_t>> surface_list;

}

#endif
