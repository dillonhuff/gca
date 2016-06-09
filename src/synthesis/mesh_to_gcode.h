#ifndef MESH_TO_GCODE_H
#define MESH_TO_GCODE_H

#include "gcode/gcode_program.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/vice.h"
#include "synthesis/tool.h"
#include "synthesis/workpiece.h"

namespace gca {

  ostream& operator<<(ostream& out, const workpiece& w);

  enum axis { X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, C_AXIS };

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
  };

  class stock_orientation {
  protected:
    const surface* left;
    const surface* right;
    const surface* bottom;

  public:
    inline point top_normal() const {
      point bn = bottom->face_orientation(bottom->front());
      point n = bn - 2*bn;
      assert(within_eps(angle_between(n, bn), 180, 0.1));
      return n;
    }

    inline point left_normal() const {
      point bn = left->face_orientation(bottom->front());
      point n = bn - 2*bn;
      assert(within_eps(angle_between(n, bn), 180, 0.1));
      return n;
    }
    
    inline const triangular_mesh& get_mesh() const
    { return left->get_parent_mesh(); }

    stock_orientation(const surface* p_left,
		      const surface* p_right,
		      const surface* p_bottom) :
      left(p_left), right(p_right), bottom(p_bottom) {}
  };

  std::vector<surface> outer_surfaces(const triangular_mesh& m);

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& m,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece w_dims);

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices);

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh);

  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w);
  

  std::vector<triangular_mesh>
  part_arrangements(const triangular_mesh& part_mesh, const vice v);

  std::vector<gcode_program>
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const vice v);
  
}

#endif
