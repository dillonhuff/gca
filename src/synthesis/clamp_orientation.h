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

    plane left_pl;
    plane right_pl;
    plane bottom_pl;

    inline const triangular_mesh& get_mesh() const
    { return left->get_parent_mesh(); }
    
  public:

    inline double surface_area() const {
      return left->surface_area() + right->surface_area() + bottom->surface_area();
    }

    inline plane base_plane() const {
      return bottom_pl;
    }

    inline plane left_plane() const {
      return left_pl;
    }

    inline plane right_plane() const {
      return right_pl;
    }
    
    inline point bottom_plane_point() const {
      return bottom_pl.pt();
    }

    inline point left_plane_point() const {
      return left_pl.pt();
    }

    inline point right_plane_point() const {
      return right_pl.pt();
    }
    
    inline point top_normal() const {
      return -1*bottom_normal();
    }

    inline point bottom_normal() const {
      return bottom_pl.normal();
    }
    
    inline point left_normal() const {
      return left_pl.normal();
    }

    inline point right_normal() const {
      return right_pl.normal();
    }

    clamp_orientation(const surface* p_left,
    		      const surface* p_right,
    		      const surface* p_bottom) :
      left(p_left), right(p_right), bottom(p_bottom),
      left_pl(point(0, 0, 1), point(0, 0, 0)),
      right_pl(point(0, 0, 1), point(0, 0, 0)),
      bottom_pl(point(0, 0, 1), point(0, 0, 0)) {

      point n = bottom->face_orientation(bottom->front());
      point p = max_point_in_dir(get_mesh(), n);
      bottom_pl = plane(n, p);

      n = left->face_orientation(left->front());
      p = max_point_in_dir(get_mesh(), n);
      left_pl = plane(n, p);

      n = right->face_orientation(right->front());
      p = max_point_in_dir(get_mesh(), n);
      right_pl = plane(n, p);
    }

    clamp_orientation() :
      left(nullptr), right(nullptr), bottom(nullptr),
      left_pl(point(0, 0, 0), point(0, 0, 0)),
      right_pl(point(0, 0, 0), point(0, 0, 0)),
      bottom_pl(point(0, 0, 0), point(0, 0, 0)) {}
      
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
