#ifndef GCA_CLAMP_ORIENTATION_H
#define GCA_CLAMP_ORIENTATION_H

#include "geometry/surface.h"
#include "synthesis/vice.h"

namespace gca {

  class clamp_orientation {
  protected:
    const surface* left;
    const surface* right;
    const surface* bottom;

  public:

    inline const surface& get_left() const { return *left; }
    inline const surface& get_right() const { return *right; }
    inline const surface& get_bottom() const { return *bottom; }

    inline plane base_plane() const {
      point n = bottom_normal();
      point p = max_point_in_dir(get_mesh(), n);
      return plane(n, p);
    }
    
    inline plane left_plane() const {
      point n = left_normal();
      point p = max_point_in_dir(get_mesh(), n);
      return plane(n, p);
    }

    inline point bottom_plane_point() const {
      const triangular_mesh& m = bottom->get_parent_mesh();
      triangle_t t = m.triangle_vertices(bottom->front());
      return m.vertex(t.v[0]);
    }
    
    inline point top_normal() const {
      point bn = bottom->face_orientation(bottom->front());
      point n = bn - 2*bn;
      assert(within_eps(angle_between(n, bn), 180, 0.1));
      return n;
    }

    inline point bottom_normal() const {
      point bn = bottom->face_orientation(bottom->front());
      return bn;
    }
    
    inline point left_normal() const {
      point bn = left->face_orientation(left->front());
      return bn;
    }
    
    inline const triangular_mesh& get_mesh() const
    { return left->get_parent_mesh(); }

    clamp_orientation(const surface* p_left,
		      const surface* p_right,
		      const surface* p_bottom) :
      left(p_left), right(p_right), bottom(p_bottom) {}

    clamp_orientation() :
      left(nullptr), right(nullptr), bottom(nullptr) {}
  };

  homogeneous_transform mating_transform(const clamp_orientation& orient,
					 const vice& v);

  triangular_mesh
  oriented_part_mesh(const clamp_orientation& orient,
		     const vice v);
  
}

#endif
