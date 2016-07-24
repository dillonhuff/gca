#ifndef GCA_CLAMP_ORIENTATION_H
#define GCA_CLAMP_ORIENTATION_H

#include "geometry/homogeneous_transformation.h"
#include "geometry/surface.h"
#include "synthesis/vice.h"

namespace gca {

  class clamp_orientation {
  protected:
    plane left_pl;
    plane right_pl;
    plane bottom_pl;

  public:

    double contact_area(const triangular_mesh& m) const;

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
      left_pl(point(0, 0, 1), point(0, 0, 0)),
      right_pl(point(0, 0, 1), point(0, 0, 0)),
      bottom_pl(point(0, 0, 1), point(0, 0, 0)) {

      auto mesh = p_left->get_parent_mesh();

      point n = p_bottom->face_orientation(p_bottom->front());
      point p = max_point_in_dir(mesh, n);
      bottom_pl = plane(n, p);

      n = p_left->face_orientation(p_left->front());
      p = max_point_in_dir(mesh, n);
      left_pl = plane(n, p);

      n = p_right->face_orientation(p_right->front());
      p = max_point_in_dir(mesh, n);
      right_pl = plane(n, p);
    }

    clamp_orientation() :
      left_pl(point(0, 0, 1), point(0, 0, 0)),
      right_pl(point(0, 0, 1), point(0, 0, 0)),
      bottom_pl(point(0, 0, 1), point(0, 0, 0)) {}
      
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
			 const std::vector<surface*>& surfaces_left,
			 const vice& v);

  clamp_orientation
  find_orientation_by_normal(const std::vector<clamp_orientation>& orients,
			     const point n);

  std::vector<clamp_orientation>
  three_orthogonal_orients(const std::vector<clamp_orientation>& orients);

  clamp_orientation
  next_orthogonal_to_all(const std::vector<clamp_orientation>& to_check,
			 const std::vector<clamp_orientation>& to_return_from);
 
}

#endif
