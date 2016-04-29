#ifndef MESH_TO_GCODE_H
#define MESH_TO_GCODE_H

#include "geometry/triangular_mesh.h"

namespace gca {

  typedef point workpiece_dimensions;

  enum tool_type { FLAT_NOSE, BALL_NOSE };
  enum axis { X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, C_AXIS };

  class vice {
  public:
    vice(double p_length, double p_width, double p_height, axis p_ax) {}
  };

  class tool {
  public:
    tool(double p_diameter, tool_type t) {}
  };

  class gcode_program {};

  class surface {
  protected:
    const triangular_mesh* parent_mesh;
    vector<index_t> tri_indexes;

  public:
    inline bool contains(index_t ind) {
      return std::binary_search(begin(tri_indexes), end(tri_indexes), ind);
    }

    inline index_t front() const {
      return tri_indexes.front();
    }

    inline point face_orientation(index_t ind) const
    { return parent_mesh->face_orientation(ind); }

    surface(const triangular_mesh* p_parent_mesh,
	    const vector<index_t>& p_tri_indexes) :
	parent_mesh(p_parent_mesh), tri_indexes(p_tri_indexes) {
      assert(tri_indexes.size() > 0);
      std::sort(begin(tri_indexes), end(tri_indexes));
    }
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

    stock_orientation(const surface* p_left,
		      const surface* p_right,
		      const surface* p_bottom) :
      left(p_left), right(p_right), bottom(p_bottom) {}
  };

  std::vector<surface> part_stable_surfaces(const triangular_mesh& m);

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& m,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece_dimensions w_dims);

  void remove_sa_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices);

}

#endif
