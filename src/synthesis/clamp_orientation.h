#ifndef GCA_CLAMP_ORIENTATION_H
#define GCA_CLAMP_ORIENTATION_H

#include "geometry/homogeneous_transformation.h"
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

    inline double surface_area() const {
      return left->surface_area() + right->surface_area() + bottom->surface_area();
    }

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

    inline plane right_plane() const {
      point n = right_normal();
      point p = max_point_in_dir(get_mesh(), n);
      return plane(n, p);
    }
    
    inline point bottom_plane_point() const {
      const triangular_mesh& m = bottom->get_parent_mesh();
      triangle_t t = m.triangle_vertices(bottom->front());
      return m.vertex(t.v[0]);
    }

    inline point left_plane_point() const {
      const triangular_mesh& m = left->get_parent_mesh();
      triangle_t t = m.triangle_vertices(left->front());
      return m.vertex(t.v[0]);
    }

    inline point right_plane_point() const {
      const triangular_mesh& m = right->get_parent_mesh();
      triangle_t t = m.triangle_vertices(right->front());
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

    inline point right_normal() const {
      point bn = right->face_orientation(right->front());
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

  homogeneous_transform mating_transform(const triangular_mesh& m,
					 const clamp_orientation& orient,
					 const vice& v);

  triangular_mesh
  oriented_part_mesh(const triangular_mesh& m,
		     const clamp_orientation& orient,
		     const vice v);

  std::vector<clamp_orientation>
  all_clamp_orientations(const std::vector<const surface*>& surfaces);
  
  void filter_stub_orientations(std::vector<clamp_orientation>& orients,
				const vice& v);
  
  std::vector<clamp_orientation>
  all_viable_clamp_orientations(const std::vector<surface>& surfaces,
				const vice& v);
  
  std::vector<clamp_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces,
			  const vice& v);

  std::vector<clamp_orientation>
  all_stable_orientations(const std::vector<const surface*>& surfaces,
			  const vice& v);

  std::vector<unsigned>
  surfaces_millable_from(const clamp_orientation& orient,
			 const std::vector<surface>& surfaces_left,
			 const vice& v);

}

#endif
